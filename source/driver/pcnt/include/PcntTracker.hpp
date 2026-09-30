#pragma once

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include <atomic>

class PcntTracker {
public:
    PcntTracker() = default;
    ~PcntTracker();

    bool init(gpio_num_t pin_pu, gpio_num_t pin_dr);
    int64_t get_total_steps() const;
    void reset();

private:
    pcnt_unit_handle_t pcnt_unit_ = nullptr;
    pcnt_channel_handle_t pcnt_chan_ = nullptr;
    
    mutable std::atomic<int64_t> accumulated_steps_{0};

    static constexpr int16_t PCNT_HIGH_LIMIT = 30000;
    static constexpr int16_t PCNT_LOW_LIMIT  = -30000;

    static bool IRAM_ATTR on_reach_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
};