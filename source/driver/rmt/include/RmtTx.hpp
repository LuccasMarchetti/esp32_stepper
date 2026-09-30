#pragma once

#include <cstdint>
#include <vector>
#include <atomic>
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "smooth_encoder.hpp"

/**
 * @brief C++ Wrapper for ESP-IDF v6.x RMT (Remote Control Transmitter) TX Driver.
 */
class RmtTx {
public:
    using CallbackFn = void (*)(void* user_arg);

    static constexpr uint32_t MAX_SYMBOL_TICKS = 32767;

    struct Config {
        gpio_num_t gpio_num = GPIO_NUM_NC;
        uint32_t resolution_hz = 1'000'000; // 1 MHz resolution (1 tick = 1 µs)
        size_t mem_block_symbols = 512;       // RMT memory block size in symbols
        uint32_t trans_queue_depth = 4;      // Max transactions queued
    };

    RmtTx();
    ~RmtTx();

    // Prevent copying to avoid resource duplication
    RmtTx(const RmtTx&) = delete;
    RmtTx& operator=(const RmtTx&) = delete;

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
     * @brief Transmits a repeated high/low pulse pattern.
     * @param high_ticks Duration of the high pulse level (in resolution ticks).
     * @param low_ticks Duration of the low pulse level (in resolution ticks).
     * @param count Number of times the high/low sequence repeats.
     * @return true if transmission started, false if busy or on error.
     */
    bool transmit_pulse_pattern(uint16_t high_ticks, uint16_t low_ticks, uint32_t count);

    /**
     * @brief Transmits an arbitrary sequence of raw RMT symbol words.
     * @param symbols Pointer to array of rmt_symbol_word_t structures.
     * @param count Number of symbol words in the array.
     * @return true if transmission started, false if busy or on error.
     */
    bool transmit_symbols(const rmt_symbol_word_t* symbols, size_t count);

    /**
     * @brief Checks if a transmission is currently in progress.
     * @return true if channel is active/busy, false if idle.
     */
    bool is_busy() const;

    /**
     * @brief Starts continuous pulse generation (infinite loop) until stopped.
     */
    bool start_continuous_pulses(uint16_t high_ticks, uint16_t low_ticks);

    /**
     * @brief Sends a buffer of RMT symbols for transmission.
     * @param tx_config Transmission configuration.
     * @return true if transmission started, false if busy or on error.
     */
    bool send_buffer(const rmt_transmit_config_t* tx_config);

    /**
     * @brief Starts an S-curve motion profile transmission using the configured RMT channel.
     * @param n1 Steps in phase 1 (Acceleration: Jerk +)
     * @param n2 Steps in phase 2 (Acceleration: Constant Acceleration)
     * @param n3 Steps in phase 3 (Acceleration: Jerk -)
     * @param n4 Steps in phase 4 (Cruise: Constant Speed)
     * @param n5 Steps in phase 5 (Deceleration: Jerk -)
     * @param n6 Steps in phase 6 (Deceleration: Constant Deceleration)
     * @param n7 Steps in phase 7 (Deceleration: Jerk +)
     * @param v_min Minimum speed (steps/s)
     * @param v_max Maximum speed (steps/s)
     * @param a_max Maximum acceleration (steps/s²)
     * @param j_max Maximum jerk (steps/s³)
     * @return true if transmission started successfully, false if busy or on error.
     */
    bool start_smooth_curve(uint32_t n_accel, uint32_t n_cruise, uint32_t n_decel,
              uint32_t start_freq, uint32_t cruise_freq, uint32_t end_freq);

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
    rmt_encoder_handle_t smooth_encoder_ = nullptr;

    bool is_enabled_{false};
    std::atomic<bool> is_busy_{false};
    CallbackFn user_callback_ = nullptr;
    void* user_arg_ = nullptr;

    std::vector<rmt_symbol_word_t> symbol_buffer_;
    rmt_symbol_word_t symbol_;

    uint32_t resolution_hz_ = 1'000'000;
};