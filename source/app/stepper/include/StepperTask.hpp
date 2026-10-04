#pragma once

#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "Q3hb220m.hpp"

/**
 * @brief Thread-safe FreeRTOS Task wrapper for the Q3HB220M Stepper Motor Controller.
 * 
 * Manages stepper motor operations asynchronously using a FreeRTOS message queue.
 * Allows independent execution alongside other system tasks (e.g., future BLDC motors).
 */
class StepperTask {
public:
    /**
     * @brief Asynchronous command types handled by the task loop.
     */
    enum class CommandType : uint8_t {
        MOVE_DELTA_ANGLE,
        MOVE_ABS_ANGLE,
        TARE,
        STOP,
        SET_DIRECTION,
        SET_SUBDIVISION_GROUP,
        SET_SK1_STEPS,
        SET_SK2_STEPS,
        SET_MOTOR_FREE,
        SET_START_SPEED,
        SET_SPEED,
        SET_ACCELERATION
    };

    /**
     * @brief Command packet payload structure.
     */
    struct Command {
        CommandType type;
        union {
            float angle_degrees;
            float start_speed_rev_s;
            float speed_rev_s;
            float acceleration_rev_s2;
            uint32_t steps_per_rev;
            Q3hb220m::Direction direction;
            Q3hb220m::SubdivisionGroup group;
            bool motor_free;
        } payload;
    };

    StepperTask();
    ~StepperTask();

    /**
     * @brief Initializes the low-level Q3HB220M hardware driver and allocates the queue.
     * @param driver_config Hardware pinout and default configuration settings.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init(const Q3hb220m::Config& driver_config);

    /**
     * @brief Creates and runs the dedicated FreeRTOS task.
     * @param priority Task priority (default: 5).
     * @param core_id CPU core affinity (default: Core 1).
     * @return true if task creation succeeded, false otherwise.
     */
    bool start(UBaseType_t priority = 5, BaseType_t core_id = 1);

    // =========================================================================
    // Asynchronous Control API (Thread-Safe / Non-Blocking)
    // =========================================================================

    /** @brief Moves the motor relative to its current position by delta_degrees. */
    bool move_delta_async(float delta_degrees);

    /** @brief Moves the motor to a target absolute angle in degrees. */
    bool move_abs_async(float abs_degrees);

    /** @brief Resets the current step position to zero. */
    bool tare_async();

    /** @brief Starts continuous rotation in the specified direction. */
    bool start_continuous_async(Q3hb220m::Direction dir);

    /** @brief Stops any ongoing motion (pulse transmission). */
    bool stop_async();

    /** 
     * @brief Dynamically changes motion direction (useful during continuous mode). 
     * @param dir New direction (CW or CCW).
     */
    bool set_direction_async(Q3hb220m::Direction dir);

    bool set_start_speed_async(float start_speed_rev_s);

    /** @brief Sets target speed as a number of RMT ticks. */
    bool set_speed_async(float speed_rev_s);

    bool set_acceleration_async(float acceleration_rev_s2);

    /** @brief Switches active hardware subdivision group between SK1 and SK2. */
    bool set_subdivision_group_async(Q3hb220m::SubdivisionGroup group);

    /** @brief Configures steps per revolution for SK1 (must match hardware table). */
    bool set_sk1_steps_async(uint32_t steps_per_rev);

    /** @brief Configures steps per revolution for SK2 (must match hardware table). */
    bool set_sk2_steps_async(uint32_t steps_per_rev);

    /** @brief De-energizes motor coils (free wheeling) or re-enables holding torque. */
    bool set_motor_free_async(bool enable_free);

    // =========================================================================
    // Synchronous Telemetry & State API (Thread-Safe Read-Only)
    // =========================================================================

    /** @brief Gets the current step position converted to degrees. */
    float get_current_angle_degrees() const;

    /** @brief Returns true if the RMT peripheral is actively generating pulses. */
    bool is_busy() const;

    /** @brief Gets the RMT resolution in Hertz. */
    uint32_t get_rmt_resolution_hz() const;

    /** @brief Gets start set angular velocity in degrees per second (°/s). */
    float get_start_speed_deg_per_sec() const;

    /** @brief Gets start set angular velocity in revolutions per second (rev/s). */
    float get_start_speed_rev_s() const;

    /** @brief Gets desired angular velocity in degrees per second (°/s). */
    float get_speed_deg_per_sec() const;

    /** @brief Gets desired angular velocity in revolutions per second (rev/s). */
    float get_speed_rev_s() const;

    /** @brief Gets set angular acceleration in degrees per second squared (°/s²). */
    float get_acceleration_deg_per_sec2() const;

    /** @brief Gets set angular acceleration in revolutions per second squared (rev/s²). */
    float get_acceleration_rev_s2() const;

    /** @brief Gets the steps-per-revolution value of the currently active group (SK1 or SK2). */
    uint32_t get_active_subdivision() const;

    /** @brief Gets the active subdivision group (SK1 or SK2). */
    Q3hb220m::SubdivisionGroup get_subdivision_group() const;

    /** @brief Gets the steps-per-revolution value of the SK1 group. */
    uint32_t get_sk1_steps() const;

    /** @brief Gets the steps-per-revolution value of the SK2 group. */
    uint32_t get_sk2_steps() const;

    /** @brief Gets the current motion direction (CW or CCW). */
    Q3hb220m::Direction get_current_direction() const;

    /** @brief Returns true if the Q3HB220M driver is powered and ready. */
    bool is_driver_ready() const;

    /** @brief Saves the current settings to NVS (Non-Volatile Storage). */
    bool save_settings_to_nvs();

private:
    static void task_entry(void* arg);
    void run();

    Q3hb220m driver_;
    QueueHandle_t command_queue_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    bool is_initialized_ = false;
};