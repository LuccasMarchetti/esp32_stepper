#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_encoder.h"
#include "esp_err.h"

#define SMOOTH_ENCODER_MAX_RAMP_STEPS 2048

// Configuração de criação do Encoder (executado 1x no setup)
typedef struct {
    uint32_t resolution_hz;  // Ex: 1000000 (1 MHz)
    uint32_t pulse_width_us; // Ex: 10 us (Largura do pulso HIGH)
} smooth_encoder_config_t;

// Parâmetros de cada movimento (atualizado a cada perfil desejado)
typedef struct {
    uint32_t n_accel;      // Passos de aceleração (0 a 512)
    uint32_t n_cruise;     // Passos de velocidade constante
    uint32_t n_decel;      // Passos de desaceleração (0 a 512)
    uint32_t start_freq;   // Frequência inicial (Hz)
    uint32_t cruise_freq;  // Frequência de cruzeiro (Hz)
    uint32_t end_freq;     // Frequência final (Hz)
} smooth_motion_config_t;

// Cria o encoder na inicialização
esp_err_t rmt_new_smooth_stepper_encoder(const smooth_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

// Prepara as rampas no buffer estático antes de transmitir (Roda na Task)
esp_err_t rmt_smooth_stepper_prepare(rmt_encoder_handle_t encoder, const smooth_motion_config_t *motion);