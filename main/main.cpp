#include "StepperTask.hpp"
#include "Cli.hpp"

extern "C" void app_main(void) {
    static StepperTask stepper_task;
    static Cli cli;

    Q3hb220m::Config config = {
        .pin_pu = GPIO_NUM_2,
        .pin_dr = GPIO_NUM_19,
        .pin_sm = GPIO_NUM_21,
        .pin_mf = GPIO_NUM_22,
        .pin_rdy = GPIO_NUM_23,
        .default_settings = {
            .active_subdivision = Q3hb220m::SubdivisionGroup::SK1,
            .sk1_steps_per_rev = 2000,
            .sk2_steps_per_rev = 10000,
            .speed_ticks = 1000
        }
    };

    if (stepper_task.init(config) && stepper_task.start()) {
        cli.init(stepper_task);
        cli.start();
    }
}