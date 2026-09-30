#pragma once

#include <cstdint>
#include <array>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "RmtTx.hpp"
#include "PcntTracker.hpp"
#include "NvsStorage.hpp"
#include <atomic>

/**
 * @brief Advanced Controller for Q3HB220M 3-Phase Stepper Motor Driver.
 */

class Q3hb220m {
public:

    // 16 valid subdivision channels (s/r) as defined in the Q3HB220M datasheet
    static constexpr std::array<uint32_t, 16> VALID_SUBDIVISIONS = {
        400, 500, 600, 800, 1000, 1200, 2000, 3000,
        4000, 5000, 6000, 10000, 12000, 20000, 30000, 60000
    };

    /**
     * @brief Checks whether a given s/r value matches one of the 16 hardware switch options.
     * @param steps_per_rev Steps per revolution (s/r) value.
     * @return true if valid, false otherwise.
     */
    static constexpr bool is_valid_subdivision(uint32_t steps_per_rev) {
        for (const auto val : VALID_SUBDIVISIONS) {
            if (val == steps_per_rev) {
                return true;
            }
        }
        return false;
    }

    enum class Direction : uint8_t {
        CW = 0,  // Clockwise
        CCW = 1  // Counter-Clockwise
    };

    enum class SubdivisionGroup : uint8_t {
        SK1 = 1, // SM pin LOW (SK1 switch bank active)
        SK2 = 0  // SM pin HIGH (SK2 switch bank active)
    };

    // Settings persisted to NVS Flash
    struct SavedSettings {
        SubdivisionGroup active_subdivision = SubdivisionGroup::SK1;
        uint32_t sk1_steps_per_rev = 2000;  // Default SK1 subdivision (s/r)
        uint32_t sk2_steps_per_rev = 10000; // Default SK2 subdivision (s/r)
        uint32_t speed_ticks = 1000;        // Default speed in RMT ticks (1 tick = 1 µs at 1 MHz)
        double position_degrees = 0.0;
    };

    struct Config {
        gpio_num_t pin_pu = GPIO_NUM_NC;    // PU: Pulse signal
        gpio_num_t pin_dr = GPIO_NUM_NC;    // DR: Direction signal
        gpio_num_t pin_sm = GPIO_NUM_NC;    // SM: Subdivision select
        gpio_num_t pin_mf = GPIO_NUM_NC;    // MF: Motor Free signal
        gpio_num_t pin_rdy = GPIO_NUM_NC;   // RDY: Ready signal (optional input)
        uint32_t min_pulse_width_us = 3;    // 100% of speed (3us = 166.67 kHz)
        SavedSettings default_settings;     // Initial defaults if Flash is empty
    };

    Q3hb220m();
    ~Q3hb220m();

    bool init(const Config& config);

    bool is_driver_ready() const;

    // Dynamic configuration methods (Auto-saved to Flash after 3s of inactivity)
    bool set_subdivision_group(SubdivisionGroup group);
    bool set_sk1_steps_per_rev(uint32_t steps_per_rev);
    bool set_sk2_steps_per_rev(uint32_t steps_per_rev);
    bool set_speed_ticks(uint32_t ticks);

    // Real-time non-persisted parameters
    void set_direction(Direction dir);
    void set_motor_free(bool enable_free);

    // Positioning and Motion Control
    bool move_delta_angle(float delta_degrees);
    bool move_absolute_angle(float abs_degrees);
    bool start_continuous(Direction dir);
    bool stop();

    void tare();

    // Query status
    bool is_busy() const;
    float get_current_angle_degrees() const;
    int64_t get_current_steps() const;
    uint32_t get_active_subdivision_value() const;
    Q3hb220m::SubdivisionGroup get_subdivision_group() const;
    uint32_t get_sk1_steps() const;
    uint32_t get_sk2_steps() const;
    bool is_continuous() const;
    Direction get_current_direction() const;
    uint32_t get_speed_ticks() const;
    float get_speed_deg_per_sec() const;
    uint32_t get_rmt_resolution_hz() const;

    // Flash manual operations
    bool save_settings_to_nvs();
    bool load_settings();

private:
    static void emergency_shutdown_callback();
    void schedule_nvs_save();
    void apply_gpio_signals_with_timing();
    uint32_t ticks_to_target_rpm(uint32_t ticks, uint32_t steps_per_rev) const;
    uint32_t calculate_pulse_ticks_from_speed() const;
    double get_raw_accumulated_degrees() const;

    Config config_;
    SavedSettings saved_settings_;

    // Runtime state (Not saved to Flash)
    Direction current_dir_ = Direction::CW;
    bool motor_free_ = false;
    double accumulated_angle_degrees_{0.0};
    bool is_continuous_ = false;

    RmtTx rmt_tx_;
    PcntTracker pcnt_tracker_;
    NvsStorage nvs_storage_{"q3hb_cfg"};
    
    bool is_initialized_ = false;
    std::atomic<bool> is_faulted_{false};

    static void IRAM_ATTR gpio_rdy_isr_handler(void* arg);

    static Q3hb220m* instance_;
};