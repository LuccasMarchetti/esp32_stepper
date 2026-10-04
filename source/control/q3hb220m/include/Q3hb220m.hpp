#pragma once

#include <cstdint>
#include <array>
#include "driver/gpio.h"
#include "esp_timer.h"
#include <atomic>

#include "stepper_profiles.h"
#include "RmtTx.hpp"
#include "PcntTracker.hpp"
#include "NvsStorage.hpp"

/**
 * @brief Advanced Controller for Q3HB220M 3-Phase Stepper Motor Driver.
 */

class Q3hb220m {
public:

    // 16 valid subdivision channels (s/r) as defined in the Q3HB220M datasheet
    static constexpr std::array<uint32_t, 16> VALID_SUBDIVISIONS = {
        60000, 30000, 20000, 12000, 10000, 6000, 5000, 4000,
        3000, 2000, 1200, 1000, 800, 600, 500, 400
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

    /**
     * @brief Adapter callback function for the David Austin motion profile.
     * @param ctx Context pointer to the motion profile.
     * @param pulse Pointer to the pulse duration.
     * @return true if a step is available, false otherwise.
     */
    static bool david_austin_adapter_cb(void* ctx, uint32_t* pulse);
    
    enum class Direction : uint8_t {
        CW = 0,  // Clockwise
        CCW = 1  // Counter-Clockwise
    };

    enum class SubdivisionGroup : uint8_t {
        SK1 = 1, // SM pin HIGH (SK1 switch bank active)
        SK2 = 0  // SM pin LOW (SK2 switch bank active)
    };

    // Settings persisted to NVS Flash
    struct SavedSettings {
        SubdivisionGroup active_subdivision = SubdivisionGroup::SK1;
        uint32_t sk1_steps_per_rev              = 2000;     // Default SK1 subdivision (s/r)
        uint32_t sk2_steps_per_rev              = 10000;    // Default SK2 subdivision (s/r)
        float start_speed_rev_s                 = 0.1f;     // Starting speed in revolutions per second
        float speed_rev_s                       = 0.8f;     // Maximum speed in revolutions per second
        float acceleration_rev_s2               = 1.0f;     // Maximum acceleration in degrees per second squared
        int32_t position_milidegrees            = 0;
    };

    struct Config {
        gpio_num_t pin_pu = GPIO_NUM_NC;        // PU: Pulse signal
        gpio_num_t pin_dr = GPIO_NUM_NC;        // DR: Direction signal
        gpio_num_t pin_sm = GPIO_NUM_NC;        // SM: Subdivision select
        gpio_num_t pin_mf = GPIO_NUM_NC;        // MF: Motor Free signal
        gpio_num_t pin_rdy = GPIO_NUM_NC;       // RDY: Ready signal (optional input)
        struct PulseTiming {
            uint32_t min_low_pulse_width_us = 3;// Minimum duration of low state of the pulse signal (datasheet requirement)
            uint32_t high_pulse_width_us = 1;   // Maximum duration of high state of the pulse signal
        } pulse_timing;
        SavedSettings default_settings;         // Initial defaults if Flash is empty
    };

    enum class CurveProfile : uint8_t {
        Trapezoidal = 0,
        S_Curve = 1
    };

    Q3hb220m();
    ~Q3hb220m();

    bool init(const Config& config);

    bool is_driver_ready() const;

    // Dynamic configuration methods (Auto-saved to Flash after 3s of inactivity)
    bool set_subdivision_group(SubdivisionGroup group);
    bool set_sk1_steps_per_rev(uint32_t steps_per_rev);
    bool set_sk1_index(uint8_t index);
    bool set_sk2_steps_per_rev(uint32_t steps_per_rev);
    bool set_sk2_index(uint8_t index);
    bool set_start_speed_rev_s(float start_speed_rev_s);
    bool set_speed_rev_s(float speed_rev_s);
    bool set_acceleration_rev_s2(float acceleration_rev_s2);

    // Real-time non-persisted parameters
    void set_direction(Direction dir);
    void set_motor_free(bool enable_free);

    // Positioning and Motion Control
    bool move_delta_angle(float delta_degrees);
    bool move_absolute_angle(float abs_degrees);
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
    float get_start_speed_rev_s() const;
    float get_speed_rev_s() const;
    float get_acceleration_rev_s2() const;
    uint32_t get_rmt_resolution_hz() const;

    // Flash manual operations
    bool save_settings_to_nvs();
    bool load_settings();

private:
    static void emergency_shutdown_callback();
    void schedule_nvs_save();
    void apply_gpio_signals_with_timing();
    uint32_t calculate_pulse_ticks_from_speed() const;
    int32_t get_accumulated_milidegrees() const;

    Config config_;
    SavedSettings saved_settings_;

    // Runtime state (Not saved to Flash)
    Direction current_dir_ = Direction::CW;
    bool motor_free_ = false;
    int32_t accumulated_milidegrees_{0};

    trapezoidal_david_austin_profile_t trapezoidal_profile_;

    RmtTx rmt_tx_;
    PcntTracker pcnt_tracker_;
    NvsStorage nvs_storage_{"q3hb_cfg"};
    
    bool is_initialized_ = false;
    std::atomic<bool> is_faulted_{false};

    static void IRAM_ATTR gpio_rdy_isr_handler(void* arg);

    static Q3hb220m* instance_;
};