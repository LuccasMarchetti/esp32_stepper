#include "smooth_encoder.hpp"
#include "esp_check.h"
#include <stdlib.h>
#include <math.h>

static const char *TAG = "smooth_encoder";

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
    uint32_t resolution_hz;
    uint32_t pulse_width_us;

    // Buffers estáticos fixos de 512 elementos na RAM
    rmt_symbol_word_t accel_table[SMOOTH_ENCODER_MAX_RAMP_STEPS];
    rmt_symbol_word_t decel_table[SMOOTH_ENCODER_MAX_RAMP_STEPS];
    rmt_symbol_word_t cruise_symbol;

    // Controle de estado da transmissão (usado na ISR)
    size_t current_step;
    uint32_t n_accel;
    uint32_t n_cruise;
    uint32_t n_decel;
    size_t total_steps;
} rmt_smooth_stepper_encoder_t;

// ISR do RMT: Apenas copia símbolos pré-calculados na RAM sem fazer contas
static size_t rmt_encode_smooth_stepper(rmt_encoder_t *encoder,
                                         rmt_channel_handle_t channel,
                                         const void *primary_data,
                                         size_t data_size,
                                         rmt_encode_state_t *ret_state)
{
    rmt_smooth_stepper_encoder_t *enc = __containerof(encoder, rmt_smooth_stepper_encoder_t, base);
    rmt_encoder_handle_t copy_enc = enc->copy_encoder;
    size_t total_bytes_encoded = 0;

    while (enc->current_step < enc->total_steps) {
        rmt_symbol_word_t sym;

        if (enc->current_step < enc->n_accel) {
            // Fase 1: Aceleração
            sym = enc->accel_table[enc->current_step];
        } else if (enc->current_step < (enc->n_accel + enc->n_cruise)) {
            // Fase 2: Cruzeiro
            sym = enc->cruise_symbol;
        } else {
            // Fase 3: Desaceleração
            size_t decel_idx = enc->current_step - (enc->n_accel + enc->n_cruise);
            sym = enc->decel_table[decel_idx];
        }

        rmt_encode_state_t session_state = RMT_ENCODING_RESET;
        size_t bytes = copy_enc->encode(copy_enc, channel, &sym, sizeof(rmt_symbol_word_t), &session_state);

        if (bytes == 0) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            break;
        }

        total_bytes_encoded += bytes;
        enc->current_step++;

        if (session_state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            break;
        }
    }

    if (enc->current_step >= enc->total_steps) {
        *ret_state = RMT_ENCODING_COMPLETE;
    }

    return total_bytes_encoded;
}

static esp_err_t rmt_smooth_stepper_reset(rmt_encoder_t *encoder) {
    rmt_smooth_stepper_encoder_t *enc = __containerof(encoder, rmt_smooth_stepper_encoder_t, base);
    enc->current_step = 0;
    return enc->copy_encoder->reset(enc->copy_encoder);
}

static esp_err_t rmt_del_smooth_stepper_encoder(rmt_encoder_t *encoder) {
    rmt_smooth_stepper_encoder_t *enc = __containerof(encoder, rmt_smooth_stepper_encoder_t, base);
    rmt_del_encoder(enc->copy_encoder);
    free(enc);
    return ESP_OK;
}

esp_err_t rmt_new_smooth_stepper_encoder(const smooth_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder) {
    ESP_RETURN_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, TAG, "argumentos invalidos");

    rmt_smooth_stepper_encoder_t *enc = static_cast<rmt_smooth_stepper_encoder_t*>(calloc(1, sizeof(rmt_smooth_stepper_encoder_t)));
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_NO_MEM, TAG, "sem memoria para encoder");

    enc->resolution_hz = config->resolution_hz;
    enc->pulse_width_us = config->pulse_width_us > 0 ? config->pulse_width_us : 10;

    enc->base.encode = rmt_encode_smooth_stepper;
    enc->base.reset = rmt_smooth_stepper_reset;
    enc->base.del = rmt_del_smooth_stepper_encoder;

    rmt_copy_encoder_config_t copy_config = {};
    esp_err_t ret = rmt_new_copy_encoder(&copy_config, &enc->copy_encoder);
    if (ret != ESP_OK) {
        free(enc);
        return ret;
    }

    *ret_encoder = &enc->base;
    return ESP_OK;
}

// Prepara e preenche as tabelas Smoothstep na RAM (Executado no contexto de TASK)
esp_err_t rmt_smooth_stepper_prepare(rmt_encoder_handle_t encoder, const smooth_motion_config_t *motion) {
    ESP_RETURN_ON_FALSE(encoder && motion, ESP_ERR_INVALID_ARG, TAG, "argumentos invalidos");
    
    if (motion->n_accel > SMOOTH_ENCODER_MAX_RAMP_STEPS || motion->n_decel > SMOOTH_ENCODER_MAX_RAMP_STEPS) {
        ESP_LOGE(TAG, "Passos de rampa excedem o maximo de 512!");
        return ESP_ERR_INVALID_ARG;
    }

    rmt_smooth_stepper_encoder_t *enc = __containerof(encoder, rmt_smooth_stepper_encoder_t, base);

    uint32_t pulse_ticks = (uint32_t)(((uint64_t)enc->resolution_hz * enc->pulse_width_us) / 1000000ULL);
    if (pulse_ticks == 0) pulse_ticks = 1;

    // 1. Gera Rampa de Aceleração (Smoothstep 3ª ordem)
    if (motion->n_accel > 0) {
        for (uint32_t i = 0; i < motion->n_accel; i++) {
            float x = (motion->n_accel > 1) ? ((float)i / (float)(motion->n_accel - 1)) : 1.0f;
            float smooth_x = x * x * (3.0f - 2.0f * x);
            float freq = motion->start_freq + smooth_x * (motion->cruise_freq - motion->start_freq);

            uint32_t period_ticks = (uint32_t)(enc->resolution_hz / freq);
            uint32_t rest_ticks = (period_ticks > pulse_ticks) ? (period_ticks - pulse_ticks) : 1;
            if (rest_ticks > 32767) rest_ticks = 32767; // Limite de hardware do RMT

            enc->accel_table[i].level0 = 1;
            enc->accel_table[i].duration0 = pulse_ticks;
            enc->accel_table[i].level1 = 0;
            enc->accel_table[i].duration1 = rest_ticks;
        }
    }

    // 2. Gera Símbolo de Cruzeiro (Velocidade constante)
    {
        uint32_t period_ticks = (uint32_t)(enc->resolution_hz / motion->cruise_freq);
        uint32_t rest_ticks = (period_ticks > pulse_ticks) ? (period_ticks - pulse_ticks) : 1;
        if (rest_ticks > 32767) rest_ticks = 32767;

        enc->cruise_symbol.level0 = 1;
        enc->cruise_symbol.duration0 = pulse_ticks;
        enc->cruise_symbol.level1 = 0;
        enc->cruise_symbol.duration1 = rest_ticks;
    }

    // 3. Gera Rampa de Desaceleração (Smoothstep 3ª ordem)
    if (motion->n_decel > 0) {
        for (uint32_t i = 0; i < motion->n_decel; i++) {
            float x = (motion->n_decel > 1) ? ((float)i / (float)(motion->n_decel - 1)) : 1.0f;
            float smooth_x = x * x * (3.0f - 2.0f * x);
            float freq = motion->cruise_freq - smooth_x * (motion->cruise_freq - motion->end_freq);

            uint32_t period_ticks = (uint32_t)(enc->resolution_hz / freq);
            uint32_t rest_ticks = (period_ticks > pulse_ticks) ? (period_ticks - pulse_ticks) : 1;
            if (rest_ticks > 32767) rest_ticks = 32767;

            enc->decel_table[i].level0 = 1;
            enc->decel_table[i].duration0 = pulse_ticks;
            enc->decel_table[i].level1 = 0;
            enc->decel_table[i].duration1 = rest_ticks;
        }
    }

    enc->n_accel = motion->n_accel;
    enc->n_cruise = motion->n_cruise;
    enc->n_decel = motion->n_decel;
    enc->total_steps = motion->n_accel + motion->n_cruise + motion->n_decel;
    enc->current_step = 0;

    return ESP_OK;
}