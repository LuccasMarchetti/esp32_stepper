#include "PcntTracker.hpp"
#include "esp_log.h"

static const char* TAG = "PCNT_STEPPER_TRACKER";

bool IRAM_ATTR PcntTracker::on_reach_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
    auto* self = static_cast<PcntTracker*>(user_ctx);
    
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) {
        self->accumulated_steps_.fetch_add(PCNT_HIGH_LIMIT, std::memory_order_relaxed);
    } else if (edata->watch_point_value == PCNT_LOW_LIMIT) {
        self->accumulated_steps_.fetch_add(PCNT_LOW_LIMIT, std::memory_order_relaxed);
    }
    return false;
}

bool PcntTracker::init(gpio_num_t pin_pu, gpio_num_t pin_dr) {
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = PCNT_LOW_LIMIT;
    unit_config.high_limit = PCNT_HIGH_LIMIT;

    if (pcnt_new_unit(&unit_config, &pcnt_unit_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit");
        return false;
    }

    pcnt_chan_config_t chan_config = {};
    chan_config.edge_gpio_num = pin_pu;
    chan_config.level_gpio_num = pin_dr;
    chan_config.flags.virt_edge_io_level = 0;
    chan_config.flags.virt_level_io_level = 0;

    if (pcnt_new_channel(pcnt_unit_, &chan_config, &pcnt_chan_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel");
        return false;
    }

    pcnt_channel_set_edge_action(pcnt_chan_, PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE);

    pcnt_channel_set_level_action(pcnt_chan_, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_add_watch_point(pcnt_unit_, PCNT_HIGH_LIMIT);
    pcnt_unit_add_watch_point(pcnt_unit_, PCNT_LOW_LIMIT);

    pcnt_event_callbacks_t cbs = {
        .on_reach = on_reach_callback,
    };
    pcnt_unit_register_event_callbacks(pcnt_unit_, &cbs, this);

    pcnt_unit_enable(pcnt_unit_);
    pcnt_unit_clear_count(pcnt_unit_);
    pcnt_unit_start(pcnt_unit_);

    ESP_LOGI(TAG, "Hardware STEPPER PCNT Pulse Tracker initialized on PU:%d DR:%d", pin_pu, pin_dr);
    return true;
}

int64_t PcntTracker::get_total_steps() const {
    int cur_count = 0;
    if (pcnt_unit_) {
        pcnt_unit_get_count(pcnt_unit_, &cur_count);
    }
    
    return accumulated_steps_.load(std::memory_order_relaxed) + static_cast<int64_t>(cur_count);
}

void PcntTracker::reset() {
    accumulated_steps_.store(0, std::memory_order_relaxed);
    if (pcnt_unit_) {
        pcnt_unit_clear_count(pcnt_unit_);
    }
}

PcntTracker::~PcntTracker() {
    if (pcnt_unit_) {
        pcnt_unit_stop(pcnt_unit_);
        pcnt_unit_disable(pcnt_unit_);
        pcnt_del_unit(pcnt_unit_);
    }
}