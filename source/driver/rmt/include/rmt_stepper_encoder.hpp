#pragma once

#include "driver/rmt_encoder.h"

typedef bool (*step_interval_cb_t)(void* ctx, uint32_t* ticks);

typedef struct {
    step_interval_cb_t cb;
    void*              ctx;
} rmt_step_source_t;

typedef struct {
    rmt_encoder_t     base;
    rmt_encoder_t*    copy_encoder;
    uint32_t          min_pulse_high_us;
    uint32_t          min_pulse_low_us;
    uint32_t          low_left;
    rmt_symbol_word_t pending;
    bool              has_pending;
} rmt_step_encoder_t;

typedef struct {
    uint32_t min_pulse_high_us;
    uint32_t min_pulse_low_us;
    uint32_t resolution_hz;
} rmt_step_encoder_config_t;

esp_err_t rmt_new_step_encoder(const rmt_step_encoder_config_t* cfg,
                               rmt_encoder_handle_t* ret);