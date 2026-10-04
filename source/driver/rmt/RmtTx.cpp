#include "esp_attr.h"
#include "esp_log.h"

#include "RmtTx.hpp"
#include "smooth_encoder.hpp"
#include "rmt_stepper_encoder.hpp"

static const char *TAG = "RmtTx";

RmtTx::RmtTx() = default;

RmtTx::~RmtTx() {
    if (step_encoder_) {
        rmt_del_encoder(step_encoder_);
    }
    if (tx_chan_) {
        rmt_disable(tx_chan_);
        rmt_del_channel(tx_chan_);
    }
}

bool RmtTx::init(const Config& config) {
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = config.gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = config.resolution_hz,
        .mem_block_symbols = config.mem_block_symbols,
        .trans_queue_depth = config.trans_queue_depth,
        .intr_priority = 0,
        .flags = {
            .invert_out = false,
            .with_dma = false,
            .allow_pd = false,
            .init_level = 0
        }
    };

    resolution_hz_ = config.resolution_hz;

    if (rmt_new_tx_channel(&tx_chan_config, &tx_chan_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel");
        return false;
    }

    if (rmt_enable(tx_chan_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel");
        return false;
    }

    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = RmtTx::on_trans_done,
    };
    rmt_tx_register_event_callbacks(tx_chan_, &cbs, this);

    return true;
}

void RmtTx::set_callback(CallbackFn callback, void* user_arg) {
    user_callback_ = callback;
    user_arg_ = user_arg;
}

bool RmtTx::is_busy() const {
    return is_busy_.load(std::memory_order_relaxed);
}

bool RmtTx::transmit_encoded_curve(bool (*cb)(void*, uint32_t*), void* profile, uint32_t min_high_pulse_us, uint32_t min_low_pulse_us) {
    if (is_busy_.load(std::memory_order_acquire)) {
        return false;
    }

    step_encoder_config_.min_pulse_high_us = min_high_pulse_us;
    step_encoder_config_.min_pulse_low_us = min_low_pulse_us;
    step_encoder_config_.resolution_hz = resolution_hz_;

    if (!step_encoder_) {
        ESP_ERROR_CHECK(rmt_new_step_encoder(&step_encoder_config_, &step_encoder_));
    }
    
    is_enabled_ = true;

    source_.cb = static_cast<step_interval_cb_t>(cb);
    source_.ctx = profile;

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = { .eot_level = 0, .queue_nonblocking = 0 }
    };

    if (rmt_transmit(tx_chan_, step_encoder_, &source_, sizeof(source_), &tx_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT transmission");
        return false;
    }

    return true;
}

bool RmtTx::stop() {
    if (tx_chan_) {
        if (is_enabled_) {
            rmt_disable(tx_chan_);
            esp_err_t err = rmt_enable(tx_chan_);
            is_enabled_ = (err == ESP_OK);
        }
        is_busy_.store(false, std::memory_order_release);
        return true;
    }
    return false;
}

uint32_t RmtTx::get_resolution_hz() const { 
    return resolution_hz_; 
}

uint32_t RmtTx::us_to_ticks(uint32_t us) const {
    if (resolution_hz_ == 0) return us;
    uint64_t ticks = (static_cast<uint64_t>(us) * resolution_hz_) / 1'000'000ULL;
    
    return static_cast<uint32_t>(ticks);
}

uint32_t RmtTx::get_max_pulse_width_us() const {
    if (resolution_hz_ == 0) return 0;
    return static_cast<uint32_t>((static_cast<uint64_t>(MAX_SYMBOL_TICKS) * 1'000'000ULL) / resolution_hz_);
}

bool IRAM_ATTR RmtTx::on_trans_done(rmt_channel_handle_t tx_chan, 
                                         const rmt_tx_done_event_data_t *edata, 
                                         void *user_ctx) {
    auto* self = static_cast<RmtTx*>(user_ctx);

    // Release busy flag atomically inside ISR
    self->is_busy_.store(false, std::memory_order_release);

    // Execute registered callback if present
    if (self->user_callback_) {
        self->user_callback_(self->user_arg_);
    }

    return false;
}