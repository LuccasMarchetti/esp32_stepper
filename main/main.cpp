#include "StepperTask.hpp"
#include "Cli.hpp"

static StepperTask stepper_task;
static Cli cli;

extern "C" void app_main(void) {
    Q3hb220m::Config config = {
        .pin_pu = GPIO_NUM_38,
        .pin_dr = GPIO_NUM_39,
        .pin_sm = GPIO_NUM_40,
        .pin_mf = GPIO_NUM_41,
        .pin_rdy = GPIO_NUM_42,
        .pulse_timing = {
            .min_low_pulse_width_us = 3,
            .high_pulse_width_us = 1
        },
        .default_settings = {
            .active_subdivision = Q3hb220m::SubdivisionGroup::SK1,
            .sk1_steps_per_rev = 2000,
            .sk2_steps_per_rev = 10000
        }
    };

    if (stepper_task.init(config) && stepper_task.start()) {
        cli.init(stepper_task);
        cli.start();
    }

    vTaskDelete(NULL);
}