#include "RmtTx.hpp"
#include "smooth_encoder.hpp"
#include "esp_log.h"

static const char *TAG = "RmtTx";

RmtTx::RmtTx() = default;

RmtTx::~RmtTx() {
    if (smooth_encoder_) {
        rmt_del_encoder(smooth_encoder_);
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

    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = RmtTx::on_trans_done,
    };
    rmt_tx_register_event_callbacks(tx_chan_, &cbs, this);

    smooth_encoder_config_t enc_cfg = {
            .resolution_hz = resolution_hz_, // 1 MHz
            .pulse_width_us = 3      // Pulso ativo de 2us
        };

    ESP_ERROR_CHECK(rmt_new_smooth_stepper_encoder(&enc_cfg, &smooth_encoder_));

    if (rmt_enable(tx_chan_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel");
        return false;
    }
    is_enabled_ = true;

    return true;
}

void RmtTx::set_callback(CallbackFn callback, void* user_arg) {
    user_callback_ = callback;
    user_arg_ = user_arg;
}

bool RmtTx::is_busy() const {
    return is_busy_.load(std::memory_order_relaxed);
}

bool RmtTx::transmit_pulse_pattern(uint16_t high_ticks, uint16_t low_ticks, uint32_t count) {
    if (is_busy_.load(std::memory_order_acquire) || count == 0) {
        return false;
    }

    // Build uniform pattern buffer
    rmt_symbol_word_t symbol = {
        .duration0 = high_ticks,
        .level0 = 1,
        .duration1 = low_ticks,
        .level1 = 0
    };

    symbol_buffer_.assign(count, symbol);

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };

    return send_buffer(&tx_config);
}

bool RmtTx::transmit_symbols(const rmt_symbol_word_t* symbols, size_t count) {
    if (is_busy_.load(std::memory_order_acquire) || symbols == nullptr || count == 0) {
        return false;
    }

    // If external symbols were passed, copy them to maintain buffer lifetime during transmission
    if (symbols != symbol_buffer_.data()) {
        symbol_buffer_.assign(symbols, symbols + count);
    }

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };

    return send_buffer(&tx_config);
}

bool RmtTx::start_continuous_pulses(uint16_t high_ticks, uint16_t low_ticks) {
    if (is_busy_.load(std::memory_order_acquire)) {
        return false;
    }

    symbol_ = {
        .duration0 = high_ticks,
        .level0 = 1,
        .duration1 = low_ticks,
        .level1 = 0
    };

    rmt_transmit_config_t tx_config = {
        .loop_count = -1,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };

    return send_buffer(&tx_config);
}

bool RmtTx::send_buffer(const rmt_transmit_config_t* tx_config) {
    if (!smooth_encoder_ || !tx_chan_) {
        return false;
    }

    if (smooth_encoder_) {
        rmt_encoder_reset(smooth_encoder_);
    }

    if (!is_enabled_) {
        esp_err_t err = rmt_enable(tx_chan_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(err));
            return false;
        }
        is_enabled_ = true;
    }

    is_busy_.store(true, std::memory_order_release);

    esp_err_t err = rmt_transmit(
        tx_chan_,
        smooth_encoder_,
        (tx_config->loop_count == -1) ? &symbol_ : symbol_buffer_.data(),
        (tx_config->loop_count == -1) ? sizeof(rmt_symbol_word_t) : symbol_buffer_.size() * sizeof(rmt_symbol_word_t),
        tx_config
    );

    if (err != ESP_OK) {
        is_busy_.store(false, std::memory_order_release);
        ESP_LOGE(TAG, "Transmission failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

bool RmtTx::start_smooth_curve(uint32_t n_accel, uint32_t n_cruise, uint32_t n_decel,
                               uint32_t start_freq, uint32_t cruise_freq, uint32_t end_freq) {

    // 1. Validação de Handles do RMT
    if (!tx_chan_ || !smooth_encoder_) {
        ESP_LOGE("RmtTx", "Erro: Canal RMT ou Encoder nao inicializado!");
        return false;
    }

    // 2. Trava de Ocupado (Busy)
    if (is_busy_.load(std::memory_order_acquire)) {
        return false;
    }

    smooth_motion_config_t motion = {
        .n_accel = n_accel,
        .n_cruise = n_cruise,
        .n_decel = n_decel,
        .start_freq = start_freq,
        .cruise_freq = cruise_freq,
        .end_freq = end_freq
    };

    // 3. Prepara as rampas na RAM
    if (rmt_smooth_stepper_prepare(smooth_encoder_, &motion) != ESP_OK) {
        return false; // is_busy_ ainda eh false aqui
    }

    // Seta busy apenas quando for realmente transmitir
    is_busy_.store(true, std::memory_order_release);

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = { .eot_level = 0, .queue_nonblocking = 0 }
    };

    // 4. Transmissão: Passa &motion e sizeof(motion) em vez de NULL e 0
    esp_err_t err = rmt_transmit(tx_chan_, smooth_encoder_, &motion, sizeof(motion), &tx_config);
    if (err != ESP_OK) {
        ESP_LOGE("RmtTx", "Erro ao transmitir RMT: %s", esp_err_to_name(err));
        is_busy_.store(false, std::memory_order_release); // Destrava em caso de falha
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