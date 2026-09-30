#include "Cli.hpp"
#include "Q3hb220m.hpp"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include <cstdio>
#include <cstring>

static const char* TAG = "CLI";
static StepperTask* s_stepper_ref = nullptr;

// Argtable3 structures for command argument parsing
static struct {
    struct arg_lit *cw;
    struct arg_lit *ccw;
    struct arg_end *end;
} dir_args;

static struct {
    struct arg_dbl *angle;
    struct arg_end *end;
} angle_args;

static struct {
    struct arg_int *ticks;
    struct arg_end *end;
} speed_ticks_args;

static struct {
    struct arg_int *steps;
    struct arg_end *end;
} steps_args;

static struct {
    struct arg_str *group;
    struct arg_end *end;
} group_args;

static struct {
    struct arg_int *state;
    struct arg_end *end;
} free_args;

Cli::~Cli() {
    if (repl_) {
        repl_->del(repl_);
    }
}

bool Cli::init(StepperTask& stepper_task) {
    if (is_initialized_) return true;

    stepper_task_ = &stepper_task;
    s_stepper_ref = &stepper_task;

    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "\033[0mesp32> ";
    repl_config.max_cmdline_length = 256;

    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    uart_config.baud_rate = 115200;

    if (esp_console_new_repl_uart(&uart_config, &repl_config, &repl_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP Console REPL.");
        return false;
    }

    esp_console_register_help_command();
    register_stepper_commands();

    is_initialized_ = true;
    return true;
}

bool Cli::start() {
    if (!is_initialized_ || !repl_) return false;
    return (esp_console_start_repl(repl_) == ESP_OK);
}

void Cli::register_stepper_commands() {
    // Command: stepper_status
    const esp_console_cmd_t status_cmd = {
        .command = "stepper_status",
        .help = "Prints detailed stepper motor telemetry, hardware subdivisions, and motion state",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            if (!s_stepper_ref) return 1;

            uint32_t current_ticks = s_stepper_ref->get_speed_ticks();
            uint32_t res_hz = s_stepper_ref->get_rmt_resolution_hz(); 
            float half_period_us = (static_cast<float>(current_ticks) * 1'000'000.0f) / static_cast<float>(res_hz);
            float pulse_freq_hz = static_cast<float>(res_hz) / (2.0f * static_cast<float>(current_ticks));

            char freq_str[32];
            if (pulse_freq_hz >= 1000.0f) {
                snprintf(freq_str, sizeof(freq_str), "%.2f kHz", pulse_freq_hz / 1000.0f);
            } else {
                snprintf(freq_str, sizeof(freq_str), "%.2f Hz", pulse_freq_hz);
            }

            auto active_group = s_stepper_ref->get_subdivision_group();
            bool is_sk1 = (active_group == Q3hb220m::SubdivisionGroup::SK1);
            uint32_t sk1_val = s_stepper_ref->get_sk1_steps();
            uint32_t sk2_val = s_stepper_ref->get_sk2_steps();
            
            auto dir = s_stepper_ref->get_current_direction();
            const char* dir_str = (dir == Q3hb220m::Direction::CW) ? "CW (Clockwise)" : "CCW (Counter-Clockwise)";

            bool is_busy = s_stepper_ref->is_busy();
            bool is_continuous = s_stepper_ref->is_continuous();
            
            const char* motion_mode = "IDLE (Stopped)";
            if (is_busy) {
                motion_mode = is_continuous ? "RUNNING (Continuous Mode)" : "MOVING (Positional Target)";
            }

            bool driver_ok = s_stepper_ref->is_driver_ready();

            printf("Hardware Status:  %s\n", driver_ok ? "READY (OK - 3.3V)" : "FAULT / UNPOWERED (0V)");
            printf("\n--- STEPPER MOTOR TELEMETRY ---\n");
            printf("Current Position: %.3f deg\n", s_stepper_ref->get_current_angle_degrees());
            printf("Direction:        %s\n", dir_str);
            printf("Pulse Timing:     %" PRIu32 " ticks (%.2f us half-period | %s)\n", 
       current_ticks, half_period_us, freq_str);
            printf("Angular Speed:    %.2f deg/s\n", s_stepper_ref->get_speed_deg_per_sec());
            printf("Subdivision:      %s [Active: %" PRIu32 " s/r] (SK1: %" PRIu32 " s/r | SK2: %" PRIu32 " s/r)\n",
                is_sk1 ? "SK1" : "SK2",
                is_sk1 ? sk1_val : sk2_val,
                sk1_val,
                sk2_val);
            printf("Motion State:     %s\n", motion_mode);
            printf("Status:           %s\n\n", is_busy ? "BUSY" : "IDLE");

            return 0;
        },
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&status_cmd);

    // Command: stepper_move <angle_deg>
    angle_args.angle = arg_dbl1(nullptr, nullptr, "<deg>", "Relative angle displacement (+ CW, - CCW)");
    angle_args.end = arg_end(1);
    const esp_console_cmd_t move_cmd = {
        .command = "stepper_move",
        .help = "Executes relative angle movement",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&angle_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, angle_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->move_delta_async(static_cast<float>(angle_args.angle->dval[0]));
            printf("Command queued: Move relative %.2f deg\n", angle_args.angle->dval[0]);
            return 0;
        },
        .argtable = &angle_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&move_cmd);

    // Command: stepper_goto <angle_deg>
    const esp_console_cmd_t goto_cmd = {
        .command = "stepper_goto",
        .help = "Moves motor to absolute target position in degrees",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&angle_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, angle_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->move_abs_async(static_cast<float>(angle_args.angle->dval[0]));
            printf("Command queued: Move absolute %.2f deg\n", angle_args.angle->dval[0]);
            return 0;
        },
        .argtable = &angle_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&goto_cmd);

    // Command: stepper_tare
    const esp_console_cmd_t tare_cmd = {
        .command = "stepper_tare",
        .help = "Resets the current motor position and angle to 0.00 deg (Tare)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            if (!s_stepper_ref) return 1;

            s_stepper_ref->tare_async();
            printf("Position tared successfully. Current angle reset to 0.00 deg.\n");
            return 0;
        },
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&tare_cmd);
    
    // Command: stepper_run <--cw | --ccw>
    dir_args.cw = arg_lit0(nullptr, "cw", "Clockwise direction");
    dir_args.ccw = arg_lit0(nullptr, "ccw", "Counter-clockwise direction");
    dir_args.end = arg_end(1);
    const esp_console_cmd_t run_cmd = {
        .command = "stepper_run",
        .help = "Starts continuous motion in specified direction",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&dir_args);
            if (nerrors != 0 || (dir_args.cw->count == 0 && dir_args.ccw->count == 0)) {
                printf("Error: Must specify either --cw or --ccw flag.\n");
                return 1;
            }
            auto dir = (dir_args.cw->count > 0) ? Q3hb220m::Direction::CW : Q3hb220m::Direction::CCW;
            s_stepper_ref->start_continuous_async(dir);
            printf("Command queued: Continuous run (%s)\n", (dir == Q3hb220m::Direction::CW) ? "CW" : "CCW");
            return 0;
        },
        .argtable = &dir_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&run_cmd);

    // Command: stepper_stop
    const esp_console_cmd_t stop_cmd = {
        .command = "stepper_stop",
        .help = "Stops ongoing stepper pulse generation immediately",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            s_stepper_ref->stop_async();
            printf("Command queued: Stop motion\n");
            return 0;
        },
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&stop_cmd);

    // Command: stepper_dir <--cw | --ccw>
    const esp_console_cmd_t dir_cmd = {
        .command = "stepper_dir",
        .help = "Changes motor direction (applies dynamically during run)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&dir_args);
            if (nerrors != 0 || (dir_args.cw->count == 0 && dir_args.ccw->count == 0)) {
                printf("Error: Must specify either --cw or --ccw flag.\n");
                return 1;
            }
            auto dir = (dir_args.cw->count > 0) ? Q3hb220m::Direction::CW : Q3hb220m::Direction::CCW;
            s_stepper_ref->set_direction_async(dir);
            printf("Command queued: Set direction (%s)\n", (dir == Q3hb220m::Direction::CW) ? "CW" : "CCW");
            return 0;
        },
        .argtable = &dir_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&dir_cmd);

    // Command: stepper_speed <ticks>
    speed_ticks_args.ticks = arg_int1(nullptr, nullptr, "<ticks>", "Half-period pulse duration in RMT ticks (min-32767)");
    speed_ticks_args.end = arg_end(1);
    const esp_console_cmd_t speed_cmd = {
        .command = "stepper_speed",
        .help = "Sets pulse half-period duration directly in RMT hardware ticks",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&speed_ticks_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, speed_ticks_args.end, argv[0]);
                return 1;
            }
            uint32_t ticks = static_cast<uint32_t>(speed_ticks_args.ticks->ival[0]);
            s_stepper_ref->set_speed_async(ticks);
            printf("Command queued: Speed set to %" PRIu32 " RMT ticks\n", ticks);
            return 0;
        },
        .argtable = &speed_ticks_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&speed_cmd);

    // Command: stepper_sub <sk1|sk2>
    group_args.group = arg_str1(nullptr, nullptr, "<sk1|sk2>", "Subdivision group");
    group_args.end = arg_end(1);
    const esp_console_cmd_t sub_cmd = {
        .command = "stepper_sub",
        .help = "Switches active hardware subdivision group (sk1 or sk2)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&group_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, group_args.end, argv[0]);
                return 1;
            }
            if (strcasecmp(group_args.group->sval[0], "sk1") == 0) {
                s_stepper_ref->set_subdivision_group_async(Q3hb220m::SubdivisionGroup::SK1);
                printf("Command queued: Active group -> SK1\n");
            } else if (strcasecmp(group_args.group->sval[0], "sk2") == 0) {
                s_stepper_ref->set_subdivision_group_async(Q3hb220m::SubdivisionGroup::SK2);
                printf("Command queued: Active group -> SK2\n");
            } else {
                printf("Error: Group must be 'sk1' or 'sk2'.\n");
                return 1;
            }
            return 0;
        },
        .argtable = &group_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&sub_cmd);

    // Command: stepper_sk1 <steps>
    steps_args.steps = arg_int1(nullptr, nullptr, "<s/r>", "Steps per revolution value from hardware table");
    steps_args.end = arg_end(1);
    const esp_console_cmd_t sk1_cmd = {
        .command = "stepper_sk1",
        .help = "Configures steps/rev for SK1",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&steps_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, steps_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->set_sk1_steps_async(static_cast<uint32_t>(steps_args.steps->ival[0]));
            printf("Command queued: Set SK1 steps -> %d s/r\n", steps_args.steps->ival[0]);
            return 0;
        },
        .argtable = &steps_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&sk1_cmd);

    // Command: stepper_sk2 <steps>
    const esp_console_cmd_t sk2_cmd = {
        .command = "stepper_sk2",
        .help = "Configures steps/rev for SK2",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&steps_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, steps_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->set_sk2_steps_async(static_cast<uint32_t>(steps_args.steps->ival[0]));
            printf("Command queued: Set SK2 steps -> %d s/r\n", steps_args.steps->ival[0]);
            return 0;
        },
        .argtable = &steps_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&sk2_cmd);

    // Command: stepper_free <1|0>
    free_args.state = arg_int1(nullptr, nullptr, "<1|0>", "1 to disable holding torque, 0 to enable");
    free_args.end = arg_end(1);
    const esp_console_cmd_t free_cmd = {
        .command = "stepper_free",
        .help = "Enables (1) or disables (0) motor free wheeling mode (MF pin)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&free_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, free_args.end, argv[0]);
                return 1;
            }
            bool enable_free = (free_args.state->ival[0] != 0);
            s_stepper_ref->set_motor_free_async(enable_free);
            printf("Command queued: Motor free -> %s\n", enable_free ? "ENABLED" : "DISABLED");
            return 0;
        },
        .argtable = &free_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&free_cmd);

    // Command: stepper_save
    const esp_console_cmd_t save_cmd = {
        .command = "stepper_save",
        .help = "Persists current motor settings to Flash NVS manually",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            if (!s_stepper_ref) return 1;
            if (s_stepper_ref->save_settings_to_nvs()) {
                printf("Settings successfully saved to Flash NVS.\n");
                return 0;
            }
            printf("Error: Failed to save settings to NVS.\n");
            return 1;
        },
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&save_cmd);
}