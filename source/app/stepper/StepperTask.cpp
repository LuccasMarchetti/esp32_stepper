#include "StepperTask.hpp"
#include "esp_log.h"

static const char* TAG = "STEPPER_TASK";
constexpr size_t QUEUE_CAPACITY = 1;

StepperTask::StepperTask() = default;

StepperTask::~StepperTask() {
    if (task_handle_) {
        vTaskDelete(task_handle_);
    }
    if (command_queue_) {
        vQueueDelete(command_queue_);
    }
}

bool StepperTask::init(const Q3hb220m::Config& driver_config) {
    if (is_initialized_) {
        return true;
    }

    command_queue_ = xQueueCreate(QUEUE_CAPACITY, sizeof(Command));
    if (!command_queue_) {
        ESP_LOGE(TAG, "Failed to create stepper command queue.");
        return false;
    }

    if (!driver_.init(driver_config)) {
        ESP_LOGE(TAG, "Failed to initialize Q3HB220M driver inside StepperTask.");
        return false;
    }

    is_initialized_ = true;
    return true;
}

bool StepperTask::start(UBaseType_t priority, BaseType_t core_id) {
    if (!is_initialized_) {
        ESP_LOGE(TAG, "Cannot start task before successful init().");
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        &StepperTask::task_entry,
        "stepper_ctrl_task",
        4096,
        this,
        priority,
        &task_handle_,
        core_id
    );

    return (result == pdPASS);
}

void StepperTask::task_entry(void* arg) {
    auto* self = static_cast<StepperTask*>(arg);
    if (self) {
        self->run();
    }
    vTaskDelete(nullptr);
}

void StepperTask::run() {
    ESP_LOGI(TAG, "Stepper motor control task active.");
    Command cmd;

    while (true) {
        // Block until a new command arrives from the queue
        if (xQueueReceive(command_queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            // RULE 1: If an angle-positioning move is active, wait until the goal is achieved
            // to avoid missing pulses or corrupting step count tracking.
            if (driver_.is_busy()) {
                while (driver_.is_busy()) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            // Execute incoming command safely
            switch (cmd.type) {
                case CommandType::SET_DIRECTION: {
                    Q3hb220m::Direction new_dir = cmd.payload.direction;
                    driver_.set_direction(new_dir);
                    break;
                }

                case CommandType::SET_START_SPEED: {
                    float new_start_speed = cmd.payload.start_speed_rev_s;
                    driver_.set_start_speed_rev_s(new_start_speed);
                    break;
                }

                case CommandType::SET_SPEED: {
                    uint32_t new_speed = cmd.payload.speed_rev_s;
                    driver_.set_speed_rev_s(new_speed);
                    break;
                }

                case CommandType::SET_ACCELERATION: {
                    float new_acceleration = cmd.payload.acceleration_rev_s2;
                    driver_.set_acceleration_rev_s2(new_acceleration);
                    break;
                }

                case CommandType::MOVE_DELTA_ANGLE: {
                    driver_.move_delta_angle(cmd.payload.angle_degrees);
                    break;
                }

                case CommandType::MOVE_ABS_ANGLE: {
                    driver_.move_absolute_angle(cmd.payload.angle_degrees);
                    break;
                }

                case CommandType::TARE: {
                    driver_.tare();
                    break;
                }

                case CommandType::STOP: {
                    driver_.stop();
                    break;
                }

                case CommandType::SET_SUBDIVISION_GROUP: {
                    driver_.set_subdivision_group(cmd.payload.group);
                    break;
                }

                case CommandType::SET_SK1_STEPS: {
                    if (cmd.payload.steps_per_rev < Q3hb220m::VALID_SUBDIVISIONS.size()) {
                        driver_.set_sk1_index(static_cast<uint8_t>(cmd.payload.steps_per_rev));
                    } else {
                        driver_.set_sk1_steps_per_rev(cmd.payload.steps_per_rev);
                    }
                    break;
                }

                case CommandType::SET_SK2_STEPS: {
                    if (cmd.payload.steps_per_rev < Q3hb220m::VALID_SUBDIVISIONS.size()) {
                        driver_.set_sk2_index(static_cast<uint8_t>(cmd.payload.steps_per_rev));
                    } else {
                        driver_.set_sk2_steps_per_rev(cmd.payload.steps_per_rev);
                    }
                    break;
                }

                case CommandType::SET_MOTOR_FREE: {
                    if (cmd.payload.motor_free && driver_.is_busy()) {
                        driver_.stop();
                    }
                    driver_.set_motor_free(cmd.payload.motor_free);
                    break;
                }
            }
        }
    }
}

// =========================================================================
// Asynchronous Command Issuers (Non-blocking Queue Push)
// =========================================================================

bool StepperTask::move_delta_async(float delta_degrees) {
    Command cmd {};
    cmd.type = CommandType::MOVE_DELTA_ANGLE;
    cmd.payload.angle_degrees = delta_degrees;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::move_abs_async(float abs_degrees) {
    Command cmd {};
    cmd.type = CommandType::MOVE_ABS_ANGLE;
    cmd.payload.angle_degrees = abs_degrees;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::tare_async() {
    Command cmd {};
    cmd.type = CommandType::TARE;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::stop_async() {
    Command cmd {};
    cmd.type = CommandType::STOP;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_direction_async(Q3hb220m::Direction dir) {
    Command cmd {};
    cmd.type = CommandType::SET_DIRECTION;
    cmd.payload.direction = dir;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_start_speed_async(float start_speed_rev_s) {
    Command cmd {};
    cmd.type = CommandType::SET_START_SPEED;
    cmd.payload.start_speed_rev_s = start_speed_rev_s;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_speed_async(float speed_rev_s) {
    Command cmd {};
    cmd.type = CommandType::SET_SPEED;
    cmd.payload.speed_rev_s = speed_rev_s;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_acceleration_async(float acceleration_rev_s2) {
    Command cmd {};
    cmd.type = CommandType::SET_ACCELERATION;
    cmd.payload.acceleration_rev_s2 = acceleration_rev_s2;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_subdivision_group_async(Q3hb220m::SubdivisionGroup group) {
    Command cmd {};
    cmd.type = CommandType::SET_SUBDIVISION_GROUP;
    cmd.payload.group = group;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_sk1_steps_async(uint32_t steps_per_rev) {
    Command cmd {};
    cmd.type = CommandType::SET_SK1_STEPS;
    cmd.payload.steps_per_rev = steps_per_rev;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_sk2_steps_async(uint32_t steps_per_rev) {
    Command cmd {};
    cmd.type = CommandType::SET_SK2_STEPS;
    cmd.payload.steps_per_rev = steps_per_rev;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

bool StepperTask::set_motor_free_async(bool enable_free) {
    Command cmd {};
    cmd.type = CommandType::SET_MOTOR_FREE;
    cmd.payload.motor_free = enable_free;
    return xQueueSend(command_queue_, &cmd, 0) == pdTRUE;
}

// =========================================================================
// Synchronous Telemetry Getters
// =========================================================================

float StepperTask::get_current_angle_degrees() const {
    return driver_.get_current_angle_degrees();
}

bool StepperTask::is_busy() const {
    return driver_.is_busy();
}

float StepperTask::get_start_speed_deg_per_sec() const {
    return driver_.get_start_speed_rev_s() * 360.0f; // Convert rev/s to deg/s
}

float StepperTask::get_start_speed_rev_s() const {
    return driver_.get_start_speed_rev_s();
}
float StepperTask::get_speed_deg_per_sec() const {
    return driver_.get_speed_rev_s() * 360.0f; // Convert rev/s to deg/s
}

float StepperTask::get_speed_rev_s() const {
    return driver_.get_speed_rev_s();
}

float StepperTask::get_acceleration_deg_per_sec2() const {
    return driver_.get_acceleration_rev_s2() * 360.0f; // Convert rev/s² to deg/s²
}

float StepperTask::get_acceleration_rev_s2() const {
    return driver_.get_acceleration_rev_s2();
}

uint32_t StepperTask::get_active_subdivision() const {
    return driver_.get_active_subdivision_value();
}

Q3hb220m::SubdivisionGroup StepperTask::get_subdivision_group() const {
    return driver_.get_subdivision_group();
}

uint32_t StepperTask::get_sk1_steps() const {
    return driver_.get_sk1_steps();
}

uint32_t StepperTask::get_sk2_steps() const {
    return driver_.get_sk2_steps();
}

Q3hb220m::Direction StepperTask::get_current_direction() const {
    return driver_.get_current_direction();
}

bool StepperTask::is_driver_ready() const { 
    return driver_.is_driver_ready(); 
}

uint32_t StepperTask::get_rmt_resolution_hz() const {
    return driver_.get_rmt_resolution_hz();
}
bool StepperTask::save_settings_to_nvs() {
    return driver_.save_settings_to_nvs();
}