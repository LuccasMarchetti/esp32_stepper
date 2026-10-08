#pragma once

#include <cstdint>
#include <vector>
#include <atomic>

#include "driver/rmt_tx.h"
#include "driver/gpio.h"

#include "rmt_stepper_encoder.hpp"

/**
 * @brief C++ Wrapper for ESP-IDF v6.x RMT (Remote Control Transmitter) TX Driver.
 */
class RmtTxDriver {
public:
    using CallbackFn = void (*)(void* user_arg);

    static constexpr uint32_t MAX_SYMBOL_TICKS = 32767;

    struct Config {
        gpio_num_t gpio_num = GPIO_NUM_NC;
        uint32_t resolution_hz = 1'000'000; // 1 MHz resolution (1 tick = 1 µs)
        size_t mem_block_symbols = 256;       // RMT memory block size in symbols
        uint32_t trans_queue_depth = 4;      // Max transactions queued
    };

    RmtTxDriver();
    ~RmtTxDriver();

    // Prevent copying to avoid resource duplication
    RmtTxDriver(const RmtTxDriver&) = delete;
    RmtTxDriver& operator=(const RmtTxDriver&) = delete;

    /**
     * @brief Initializes the RMT TX channel and copy encoder.
     * @param config Driver configuration parameters.
     * @return true on success, false otherwise.
     */
    bool init(const Config& config);

    /**
     * @brief Registers a completion callback function called from ISR context.
     * @param callback Function pointer executed when transmission finishes.
     * @param user_arg Optional pointer passed to the callback.
     */
    void set_callback(CallbackFn callback, void* user_arg = nullptr);

    /**
     * @brief Checks if a transmission is currently in progress.
     * @return true if channel is active/busy, false if idle.
     */
    bool is_busy() const;

    /**
     * @brief Transmits an encoded S-curve motion profile.
     * @param cb Callback function to be called upon completion.
     * @param profile Pointer to the S-curve profile data.
     * @param min_high_pulse_us Minimum high pulse width in microseconds.
     * @param min_low_pulse_us Minimum low pulse width in microseconds.
     * @return true if transmission started, false if busy or on error.
     */
    bool transmit_encoded_curve(bool (*cb)(void*, uint32_t*), void* profile, uint32_t min_high_pulse_us, uint32_t min_low_pulse_us);

    /**
     * @brief Stops any ongoing transmission immediately.
     */
    bool stop();

    /**
     * @brief Returns the configured RMT resolution in Hz.
     */
    uint32_t get_resolution_hz() const;

    /**
     * @brief Converts microseconds to RMT ticks based on the configured resolution.
     * @param us Duration in microseconds.
     * @return Equivalent duration in RMT ticks.
     */
    uint32_t us_to_ticks(uint32_t us) const;

    /**
     * @brief Returns the maximum pulse width in microseconds based on the configured resolution.
     * @return Maximum pulse width in microseconds.
     */
    uint32_t get_max_pulse_width_us() const;

private:
    static bool IRAM_ATTR on_trans_done(rmt_channel_handle_t tx_chan, 
                                            const rmt_tx_done_event_data_t *edata, 
                                            void *user_ctx);

    rmt_channel_handle_t tx_chan_ = nullptr;
    rmt_encoder_handle_t step_encoder_ = nullptr;
    rmt_step_encoder_config_t step_encoder_config_ = {};
    rmt_step_source_t source_ = {};

    bool is_enabled_{false};
    std::atomic<bool> is_busy_{false};
    CallbackFn user_callback_ = nullptr;
    void* user_arg_ = nullptr;

    uint32_t resolution_hz_ = 1'000'000;
};