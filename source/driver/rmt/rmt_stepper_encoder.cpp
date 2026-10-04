#include "rmt_stepper_encoder.hpp"
#include "esp_attr.h"
#include <sys/cdefs.h>   // __containerof

#define RMT_MAX_SYMBOL_TICKS 32767

static IRAM_ATTR bool build_next_symbol(rmt_step_encoder_t* e, const rmt_step_source_t* src)
{
    rmt_symbol_word_t s = {};

    if (e->low_left == 0) 
    {
        uint32_t c;
        if (!src->cb(src->ctx, &c)) return false;

        const uint32_t c_min = e->min_pulse_high_us + e->min_pulse_low_us;
        if (c < c_min) c = c_min;

        e->low_left = c - e->min_pulse_high_us;

        s.level0 = 1; s.duration0 = e->min_pulse_high_us;
        if (e->low_left <= RMT_MAX_SYMBOL_TICKS) {
            s.level1 = 0; s.duration1 = (e->low_left < e->min_pulse_low_us) ? e->min_pulse_low_us : e->low_left;
            e->low_left = 0;
        } else {
            s.level1 = 0; s.duration1 = RMT_MAX_SYMBOL_TICKS;
            e->low_left -= RMT_MAX_SYMBOL_TICKS;
        }
    } else 
    {
        if (e->low_left <= 2 * RMT_MAX_SYMBOL_TICKS) 
        {
            s.level0 = 0; s.duration0 = e->low_left / 2;
            s.level1 = 0; s.duration1 = e->low_left - s.duration0;
            e->low_left = 0;
        } else 
        {
            e->low_left -= 2 * RMT_MAX_SYMBOL_TICKS;
            s.level0 = 0; s.duration0 = RMT_MAX_SYMBOL_TICKS;
            s.level1 = 0; s.duration1 = RMT_MAX_SYMBOL_TICKS;
        }
    }

    e->pending = s;
    return true;
}

static IRAM_ATTR size_t step_encode(rmt_encoder_t* enc, rmt_channel_handle_t ch,
                                    const void* primary, size_t size,
                                    rmt_encode_state_t* ret_state)
{
    auto* e   = __containerof(enc, rmt_step_encoder_t, base);
    auto* src = static_cast<const rmt_step_source_t*>(primary);

    int    state = RMT_ENCODING_RESET;
    size_t n     = 0;

    while (true) {
        if (!e->has_pending) {
            if (!build_next_symbol(e, src)) {
                state |= RMT_ENCODING_COMPLETE;
                break;
            }
            e->has_pending = true;
        }

        rmt_encode_state_t session = RMT_ENCODING_RESET;
        n += e->copy_encoder->encode(e->copy_encoder, ch,
                                     &e->pending, sizeof(e->pending), &session);

        if (session & RMT_ENCODING_COMPLETE) e->has_pending = false;
        if (session & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            break;
        }
    }
    *ret_state = static_cast<rmt_encode_state_t>(state);
    return n;
}

static IRAM_ATTR esp_err_t step_reset(rmt_encoder_t* enc)
{
    auto* e = __containerof(enc, rmt_step_encoder_t, base);
    rmt_encoder_reset(e->copy_encoder);
    e->has_pending = false;
    e->low_left    = 0;
    return ESP_OK;
}

static esp_err_t step_del(rmt_encoder_t* enc)
{
    auto* e = __containerof(enc, rmt_step_encoder_t, base);
    rmt_del_encoder(e->copy_encoder);
    free(e);
    return ESP_OK;
}

esp_err_t rmt_new_step_encoder(const rmt_step_encoder_config_t* cfg,
                               rmt_encoder_handle_t* ret)
{
    if (!cfg || !ret || cfg->resolution_hz == 0) return ESP_ERR_INVALID_ARG;

    if (cfg->min_pulse_high_us > RMT_MAX_SYMBOL_TICKS) return ESP_ERR_INVALID_ARG;
    if (cfg->min_pulse_low_us > RMT_MAX_SYMBOL_TICKS) return ESP_ERR_INVALID_ARG;

    auto* e = static_cast<rmt_step_encoder_t*>(rmt_alloc_encoder_mem(sizeof(rmt_step_encoder_t)));
    if (!e) return ESP_ERR_NO_MEM;

    e->base.encode    = step_encode;
    e->base.reset     = step_reset;
    e->base.del       = step_del;
    e->low_left       = 0;
    e->min_pulse_high_us  = cfg->min_pulse_high_us;
    e->min_pulse_low_us   = cfg->min_pulse_low_us;

    rmt_copy_encoder_config_t copy_cfg = {};
    esp_err_t err = rmt_new_copy_encoder(&copy_cfg, &e->copy_encoder);
    if (err != ESP_OK) { free(e); return err; }

    step_reset(&e->base);
    *ret = &e->base;
    return ESP_OK;
}