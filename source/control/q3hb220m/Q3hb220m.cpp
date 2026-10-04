#include <stdint.h>
#include <cmath>
#include <algorithm>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"

#include "smooth_encoder.hpp"
#include "stepper_profiles.h"
#include "Q3hb220m.hpp"

static const char* TAG = "Q3HB220M";
Q3hb220m* Q3hb220m::instance_ = nullptr;

static float normalize_angle_360(float angle) {
    float norm = std::fmod(angle, 360.0f);
    if (norm < 0.0f) {
        norm += 360.0f;
    }
    return norm;
}

static float get_shortest_delta_degrees(float current_angle, float target_angle) {
    float curr_norm = normalize_angle_360(current_angle);
    float target_norm = normalize_angle_360(target_angle);

    float delta = target_norm - curr_norm;

    if (delta > 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    return delta;
}

bool IRAM_ATTR Q3hb220m::david_austin_adapter_cb(void* ctx, uint32_t* pulse)
{
    auto* p = static_cast<trapezoidal_david_austin_profile_t*>(ctx);

    bool has_step = trapezoidal_david_austin_profile_next_step(p);

    if (has_step) {
        *pulse = p->c;
    }

    return has_step;
}

Q3hb220m::Q3hb220m() {
    instance_ = this;
}

Q3hb220m::~Q3hb220m() {
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void Q3hb220m::emergency_shutdown_callback() {
    // Cut off motor coil current immediately upon brownout or reset
    if (instance_ && instance_->config_.pin_mf != GPIO_NUM_NC) {
        gpio_set_level(instance_->config_.pin_mf, 1);
    }
}

bool Q3hb220m::init(const Config& config) {
    config_ = config;

    // Validate default configuration passed by caller
    if (!is_valid_subdivision(config_.default_settings.sk1_steps_per_rev) ||
        !is_valid_subdivision(config_.default_settings.sk2_steps_per_rev)) {
        ESP_LOGE(TAG, "Invalid default subdivision values in Config. Must match hardware table.");
        return false;
    }

    NvsStorage::init_default();

    if (config_.pin_rdy != GPIO_NUM_NC) {
        gpio_config_t rdy_conf = {
            .pin_bit_mask = (1ULL << config_.pin_rdy),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_ANYEDGE
        };
        gpio_config(&rdy_conf);

        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
            return false;
        }

        gpio_isr_handler_add(config_.pin_rdy, Q3hb220m::gpio_rdy_isr_handler, this);

        int initial_level = gpio_get_level(config_.pin_rdy);
        is_faulted_.store(initial_level == 0, std::memory_order_relaxed);
        
        if (initial_level == 0) {
            ESP_LOGW(TAG, "Driver initialized in FAULT or UNPOWERED state (RDY pin is LOW)");
        }
    }

    // Configure control GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config_.pin_dr) | 
                        (1ULL << config_.pin_sm) | 
                        (1ULL << config_.pin_mf),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_conf) != ESP_OK) {
        return false;
    }

    // Load settings from NVS or set defaults
    if (load_settings() && 
        is_valid_subdivision(saved_settings_.sk1_steps_per_rev) &&
        is_valid_subdivision(saved_settings_.sk2_steps_per_rev)) {
        
        accumulated_milidegrees_ = saved_settings_.position_milidegrees;
    } else {
        ESP_LOGW(TAG, "Loaded invalid or missing NVS settings. Falling back to defaults.");
        saved_settings_ = config_.default_settings;
        accumulated_milidegrees_ = saved_settings_.position_milidegrees;
        save_settings_to_nvs();
    }

    // Initialize RMT driver for PU (Pulse) pin
    RmtTx::Config rmt_config = {
        .gpio_num = config_.pin_pu,
        .resolution_hz = 1'000'000, // 1 MHz resolution (1 tick = 1 µs)
    };
    if (!rmt_tx_.init(rmt_config)) {
        return false;
    }

    if (!pcnt_tracker_.init(config_.pin_pu, config_.pin_dr)) {
        ESP_LOGE(TAG, "Failed to initialize hardware PCNT step counter");
        return false;
    }

    apply_gpio_signals_with_timing();
    esp_register_shutdown_handler(&Q3hb220m::emergency_shutdown_callback);

    is_initialized_ = true;
    return true;
}

bool Q3hb220m::is_driver_ready() const {
    if (config_.pin_rdy == GPIO_NUM_NC) {
        return true;
    }
    
    return !is_faulted_.load(std::memory_order_relaxed) && (gpio_get_level(config_.pin_rdy) == 1);
}

void Q3hb220m::apply_gpio_signals_with_timing() {
    gpio_set_level(config_.pin_dr, static_cast<uint32_t>(current_dir_));
    gpio_set_level(config_.pin_sm, static_cast<uint32_t>(saved_settings_.active_subdivision));
    gpio_set_level(config_.pin_mf, motor_free_ ? 1 : 0);

    // Datasheet requirement: DR, SM, MF setup time must be > 2.5us before PU pulse edge
    esp_rom_delay_us(3);
}

uint32_t Q3hb220m::get_active_subdivision_value() const {
    return (saved_settings_.active_subdivision == SubdivisionGroup::SK1) 
           ? saved_settings_.sk1_steps_per_rev 
           : saved_settings_.sk2_steps_per_rev;
}

// Persisted Setting Modifiers (Triggers 3s NVS Timer)
bool Q3hb220m::set_subdivision_group(SubdivisionGroup group) {
    if (!is_driver_ready()) {
        ESP_LOGE(TAG, "Action blocked: Driver RDY is LOW (Fault or Unpowered)");
        stop();
        return false;
    }

    accumulated_milidegrees_ = get_accumulated_milidegrees();

    pcnt_tracker_.reset();

    saved_settings_.active_subdivision = group;
    apply_gpio_signals_with_timing();

    return true;
}

bool Q3hb220m::set_sk1_steps_per_rev(uint32_t steps_per_rev) {
    if (!is_valid_subdivision(steps_per_rev)) {
        ESP_LOGE(TAG, "Rejected SK1 setting: %lu s/r. Value not supported by hardware table.", steps_per_rev);
        return false;
    }
    saved_settings_.sk1_steps_per_rev = steps_per_rev;
    return true;
}

bool Q3hb220m::set_sk1_index(uint8_t index) {
    if (index >= VALID_SUBDIVISIONS.size()) {
        ESP_LOGE(TAG, "Rejected SK1 index: %X. Must be in range [0, F].", index);
        return false;
    }
    saved_settings_.sk1_steps_per_rev = VALID_SUBDIVISIONS[index];
    ESP_LOGI(TAG, "SK1 subdivision set to %u s/r (index %X)", VALID_SUBDIVISIONS[index], index);
    return true;
}

bool Q3hb220m::set_sk2_index(uint8_t index) {
    if (index >= VALID_SUBDIVISIONS.size()) {
        ESP_LOGE(TAG, "Rejected SK2 index: %X. Must be in range [0, F].", index);
        return false;
    }
    saved_settings_.sk2_steps_per_rev = VALID_SUBDIVISIONS[index];
    ESP_LOGI(TAG, "SK2 subdivision set to %u s/r (index %X)", VALID_SUBDIVISIONS[index], index);
    return true;
}

bool Q3hb220m::set_sk2_steps_per_rev(uint32_t steps_per_rev) {
    if (!is_valid_subdivision(steps_per_rev)) {
        ESP_LOGE(TAG, "Rejected SK2 setting: %lu s/r. Value not supported by hardware table.", steps_per_rev);
        return false;
    }
    saved_settings_.sk2_steps_per_rev = steps_per_rev;
    return true;
}

// Real-Time Non-Persisted Modifiers
void Q3hb220m::set_direction(Direction dir) {
    if (!is_driver_ready()) {
        ESP_LOGE(TAG, "Action blocked: Driver RDY is LOW (Fault or Unpowered)");
        stop();
        return;
    }
    current_dir_ = dir;
    gpio_set_level(config_.pin_dr, static_cast<uint32_t>(dir));
    esp_rom_delay_us(10); // Datasheet signal setup time requirement
}

bool Q3hb220m::set_start_speed_rev_s(float start_speed_rev_s) {
    if (start_speed_rev_s < 0.0f || start_speed_rev_s > saved_settings_.speed_rev_s) {
        ESP_LOGE(TAG, "Rejected start speed setting: %.2f rev/s.", 
                 start_speed_rev_s);
        return false;
    }
    saved_settings_.start_speed_rev_s = start_speed_rev_s;
    return true;
}

bool Q3hb220m::set_speed_rev_s(float speed_rev_s) {
    if (speed_rev_s < 0.0f || speed_rev_s < saved_settings_.start_speed_rev_s) {
        ESP_LOGE(TAG, "Rejected speed setting: %.2f rev/s.", 
                 speed_rev_s);
        return false;
    }
    saved_settings_.speed_rev_s = speed_rev_s;
    return true;
}

bool Q3hb220m::set_acceleration_rev_s2(float acceleration_rev_s2) {
    if (acceleration_rev_s2 < 0.0f) {
        ESP_LOGE(TAG, "Rejected acceleration setting: %.2f rev/s².", 
                 acceleration_rev_s2);
        return false;
    }
    saved_settings_.acceleration_rev_s2 = acceleration_rev_s2;
    return true;
}

void Q3hb220m::set_motor_free(bool enable_free) {
    if (!is_driver_ready()) {
        ESP_LOGE(TAG, "Action blocked: Driver RDY is LOW (Fault or Unpowered)");
        stop();
        return;
    }

    motor_free_ = enable_free;
    gpio_set_level(config_.pin_mf, enable_free ? 1 : 0);
    esp_rom_delay_us(10);
}

// Motion Control
bool Q3hb220m::move_delta_angle(float delta_degrees) {
    if (!is_driver_ready()) {
        ESP_LOGE(TAG, "Action blocked: Driver RDY is LOW (Fault or Unpowered)");
        stop();
        return false;
    }

    if (!is_initialized_ || motor_free_ || rmt_tx_.is_busy()) {
        return false;
    }

    Direction dir = (delta_degrees >= 0.0f) ? Direction::CCW : Direction::CW;
    set_direction(dir);

    uint32_t steps_per_rev = get_active_subdivision_value();
    uint32_t total_steps = static_cast<uint32_t>(std::round((std::abs(delta_degrees) / 360.0f) * steps_per_rev));

    if (total_steps == 0) return true;

    trapezoidal_profile_ = plan_trapezoidal_david_austin_motion(total_steps, 
        saved_settings_.start_speed_rev_s, 
        saved_settings_.speed_rev_s, 
        saved_settings_.acceleration_rev_s2, 
        steps_per_rev, 
        rmt_tx_.get_resolution_hz());

    apply_gpio_signals_with_timing();

    return rmt_tx_.transmit_encoded_curve(david_austin_adapter_cb, (void*)&trapezoidal_profile_, 1, 3);
}

bool Q3hb220m::move_absolute_angle(float abs_degrees) {
    float current_angle = get_current_angle_degrees();
    float shortest_delta = get_shortest_delta_degrees(current_angle, abs_degrees);

    return move_delta_angle(shortest_delta);
}

void Q3hb220m::tare() {
    accumulated_milidegrees_ = 0;
    pcnt_tracker_.reset();
}

bool Q3hb220m::stop() {
    return rmt_tx_.stop();
}

bool Q3hb220m::is_busy() const {
    return rmt_tx_.is_busy();
}

int64_t Q3hb220m::get_current_steps() const {
    return pcnt_tracker_.get_total_steps();
}

int32_t Q3hb220m::get_accumulated_milidegrees() const {
    uint32_t steps_per_rev = get_active_subdivision_value();
    if (steps_per_rev == 0) return accumulated_milidegrees_;

    int64_t current_pcnt_steps = pcnt_tracker_.get_total_steps();
    
    int32_t session_milidegrees = static_cast<int32_t>(current_pcnt_steps * (360 * 1000 / steps_per_rev));

    return accumulated_milidegrees_ + session_milidegrees;
}

float Q3hb220m::get_current_angle_degrees() const {
    return normalize_angle_360(static_cast<float>(get_accumulated_milidegrees()/1000.0f));
}

Q3hb220m::SubdivisionGroup Q3hb220m::get_subdivision_group() const { 
    return saved_settings_.active_subdivision; 
}

uint32_t Q3hb220m::get_sk1_steps() const { 
    return saved_settings_.sk1_steps_per_rev; 
}

uint32_t Q3hb220m::get_sk2_steps() const { 
    return saved_settings_.sk2_steps_per_rev; 
}

Q3hb220m::Direction Q3hb220m::get_current_direction() const { 
    return current_dir_; 
}

float Q3hb220m::get_start_speed_rev_s() const {
    return saved_settings_.start_speed_rev_s;
}

float Q3hb220m::get_speed_rev_s() const {
    return saved_settings_.speed_rev_s;
}

float Q3hb220m::get_acceleration_rev_s2() const {
    return saved_settings_.acceleration_rev_s2;
}

uint32_t Q3hb220m::get_rmt_resolution_hz() const {
    return rmt_tx_.get_resolution_hz();
}

void IRAM_ATTR Q3hb220m::gpio_rdy_isr_handler(void* arg) {
    auto* instance = static_cast<Q3hb220m*>(arg);

    int level = gpio_get_level(instance->config_.pin_rdy);

    if (level == 0) {
        instance->is_faulted_.store(true, std::memory_order_relaxed);
        instance->stop();
    } else {
        instance->is_faulted_.store(false, std::memory_order_relaxed);
    }
}

bool Q3hb220m::save_settings_to_nvs() {
    saved_settings_.position_milidegrees = get_accumulated_milidegrees();
    return nvs_storage_.write_blob("settings", &saved_settings_, sizeof(SavedSettings));
}

bool Q3hb220m::load_settings() {
    return nvs_storage_.read_blob("settings", &saved_settings_, sizeof(SavedSettings));
}