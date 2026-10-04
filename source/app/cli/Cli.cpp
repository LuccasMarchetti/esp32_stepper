#include "argtable3/argtable3.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <string>
#include "linenoise/linenoise.h"

#include "soc/soc_caps.h"
#include "esp_console.h"
#include "esp_log.h"

#include "Q3hb220m.hpp"
#include "StepperTask.hpp"
#include "Cli.hpp"

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
} move_angle_args;

static struct {
    struct arg_dbl *angle;
    struct arg_end *end;
} goto_angle_args;

static struct {
    struct arg_dbl *rev_s;
    struct arg_end *end;
} start_speed_args;

static struct {
    struct arg_dbl *rev_s;
    struct arg_end *end;
} speed_args;

static struct {
    struct arg_dbl *rev_s2;
    struct arg_end *end;
} accel_args;

static struct {
    struct arg_str *input;
    struct arg_end *end;
} sk1_steps_args;

static struct {
    struct arg_str *input;
    struct arg_end *end;
} sk2_steps_args;

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

#if SOC_USB_SERIAL_JTAG_SUPPORTED
    esp_console_dev_usb_serial_jtag_config_t usb_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    if (esp_console_new_repl_usb_serial_jtag(&usb_config, &repl_config, &repl_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP Console REPL in USB mode.");
        return false;
    }
#else
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    uart_config.baud_rate = 115200;

    if (esp_console_new_repl_uart(&uart_config, &repl_config, &repl_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP Console REPL in UART mode.");
        return false;
    }
#endif

    ESP_ERROR_CHECK(esp_console_register_help_command());
    register_stepper_commands();

    is_initialized_ = true;
    return true;
}

bool Cli::start() {
    if (!is_initialized_ || !repl_) return false;
    linenoiseSetDumbMode(0);
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

            auto active_group = s_stepper_ref->get_subdivision_group();
            bool is_sk1 = (active_group == Q3hb220m::SubdivisionGroup::SK1);
            uint32_t sk1_val = s_stepper_ref->get_sk1_steps();
            uint32_t sk2_val = s_stepper_ref->get_sk2_steps();
            
            auto dir = s_stepper_ref->get_current_direction();
            const char* dir_str = (dir == Q3hb220m::Direction::CW) ? "CW (Clockwise)" : "CCW (Counter-Clockwise)";

            bool is_busy = s_stepper_ref->is_busy();
            
            const char* motion_mode = "IDLE (Stopped)";
            if (is_busy) {
                motion_mode = "MOVING (Positional Target)";
            }

            bool driver_ok = s_stepper_ref->is_driver_ready();

            printf("Hardware Status:  %s\n", driver_ok ? "READY (OK - 3.3V)" : "FAULT / UNPOWERED (0V)");
            printf("\n--- STEPPER MOTOR TELEMETRY ---\n");
            printf("Current Position: %.3f deg\n", s_stepper_ref->get_current_angle_degrees());
            printf("Direction:        %s\n", dir_str);
            printf("Motion params:\n");
            printf("\tStart Speed: %.2f rev/s (%.2f deg/s)\n", s_stepper_ref->get_start_speed_rev_s(), s_stepper_ref->get_start_speed_deg_per_sec());
            printf("\tSpeed: %.2f rev/s (%.2f deg/s)\n", s_stepper_ref->get_speed_rev_s(), s_stepper_ref->get_speed_deg_per_sec());
            printf("\tAccel: %.2f rev/s² (%.2f deg/s²)\n",
                s_stepper_ref->get_acceleration_rev_s2(),
                s_stepper_ref->get_acceleration_deg_per_sec2());
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
    move_angle_args.angle = arg_dbl1(nullptr, nullptr, "<deg>", "Relative angle displacement (+ CW, - CCW)");
    move_angle_args.end = arg_end(1);
    const esp_console_cmd_t move_cmd = {
        .command = "stepper_move",
        .help = "Executes relative angle movement",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&move_angle_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, move_angle_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->move_delta_async(static_cast<float>(move_angle_args.angle->dval[0]));
            printf("Command queued: Move relative %.2f deg\n", move_angle_args.angle->dval[0]);
            return 0;
        },
        .argtable = &move_angle_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&move_cmd);

    // Command: stepper_goto <angle_deg>
    goto_angle_args.angle = arg_dbl1(nullptr, nullptr, "<deg>", "Absolute target angle in degrees");
    goto_angle_args.end = arg_end(1);
    const esp_console_cmd_t goto_cmd = {
        .command = "stepper_goto",
        .help = "Moves motor to absolute target position in degrees",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&goto_angle_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, goto_angle_args.end, argv[0]);
                return 1;
            }
            s_stepper_ref->move_abs_async(static_cast<float>(goto_angle_args.angle->dval[0]));
            printf("Command queued: Move absolute %.2f deg\n", goto_angle_args.angle->dval[0]);
            return 0;
        },
        .argtable = &goto_angle_args,
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
    dir_args.cw = arg_lit0(nullptr, "cw", "Set direction to Clockwise");
    dir_args.ccw = arg_lit0(nullptr, "ccw", "Set direction to Counter-Clockwise");
    dir_args.end = arg_end(2);
    
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

    // Command: stepper_start_speed <revs/s>
    start_speed_args.rev_s = arg_dbl1(nullptr, nullptr, "<revs/s>", "Stepper start speed in revolutions per second");
    start_speed_args.end = arg_end(1);
    const esp_console_cmd_t start_speed_cmd = {
        .command = "stepper_start_speed",
        .help = "Sets start speed in revolutions per second (rev/s)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&start_speed_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, start_speed_args.end, argv[0]);
                return 1;
            }
            float rev_s = static_cast<float>(start_speed_args.rev_s->dval[0]);
            s_stepper_ref->set_start_speed_async(rev_s);
            printf("Command queued: Speed set to %.2f rev/s\n", rev_s);
            return 0;
        },
        .argtable = &start_speed_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&start_speed_cmd);

    // Command: stepper_speed <revs/s>
    speed_args.rev_s = arg_dbl1(nullptr, nullptr, "<revs/s>", "Speed in revolutions per second");
    speed_args.end = arg_end(1);
    const esp_console_cmd_t speed_cmd = {
        .command = "stepper_speed",
        .help = "Sets target speed in revolutions per second (rev/s)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&speed_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, speed_args.end, argv[0]);
                return 1;
            }
            float rev_s = static_cast<float>(speed_args.rev_s->dval[0]);
            s_stepper_ref->set_speed_async(rev_s);
            printf("Command queued: Speed set to %.2f rev/s\n", rev_s);
            return 0;
        },
        .argtable = &speed_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&speed_cmd);

    // Command: stepper_accel <revs/s^2>
    accel_args.rev_s2 = arg_dbl1(nullptr, nullptr, "<revs/s^2>", "Acceleration in revolutions per second squared");
    accel_args.end = arg_end(1);
    const esp_console_cmd_t accel_cmd = {
        .command = "stepper_accel",
        .help = "Sets acceleration in revolutions per second squared (rev/s^2)",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&accel_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, accel_args.end, argv[0]);
                return 1;
            }
            float rev_s2 = static_cast<float>(accel_args.rev_s2->dval[0]);
            s_stepper_ref->set_acceleration_async(rev_s2);
            printf("Command queued: Acceleration set to %.2f rev/s^2\n", rev_s2);
            return 0;
        },
        .argtable = &accel_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&accel_cmd);

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

    // Command: stepper_sk1 <steps or index>
    sk1_steps_args.input = arg_str1(nullptr, nullptr, "<s/r or i>", "Steps per revolution value from hardware table or index 0-F");
    sk1_steps_args.end = arg_end(1);
    const esp_console_cmd_t sk1_cmd = {
        .command = "stepper_sk1",
        .help = "Configures steps/rev for SK1",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&sk1_steps_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, sk1_steps_args.end, argv[0]);
                return 1;
            }

            const char* str = sk1_steps_args.input->sval[0];
            size_t len = strlen(str);
            uint32_t parsed_val = 0;

            if (len == 1) {
                if (std::isxdigit(static_cast<unsigned char>(str[0]))) {
                    parsed_val = static_cast<uint32_t>(std::strtoul(str, nullptr, 16));
                } else {
                    printf("Error: Single-character input '%s' is not a valid hex digit (0-F).\n", str);
                    return 1;
                }
            } else if (len > 1) {
                for (size_t i = 0; i < len; ++i) {
                    if (!std::isdigit(static_cast<unsigned char>(str[i]))) {
                        printf("Error: Invalid input '%s'. Multi-character inputs must be decimal integers.\n", str);
                        return 1;
                    }
                }
                parsed_val = static_cast<uint32_t>(std::strtoul(str, nullptr, 10));
            } else {
                printf("Error: Empty input.\n");
                return 1;
            }

            s_stepper_ref->set_sk1_steps_async(parsed_val);
            printf("Command queued: Set SK1 value -> %lu (0x%X)\n", parsed_val, (unsigned int)parsed_val);
            return 0;
        },
        .argtable = &sk1_steps_args,
        .func_w_context = nullptr,
        .context = nullptr
    };
    esp_console_cmd_register(&sk1_cmd);

    // Command: stepper_sk2 <steps or index>
    sk2_steps_args.input = arg_str1(nullptr, nullptr, "<s/r or i>", "Steps per revolution value from hardware table or index 0-F");
    sk2_steps_args.end = arg_end(1);
    const esp_console_cmd_t sk2_cmd = {
        .command = "stepper_sk2",
        .help = "Configures steps/rev for SK2",
        .hint = nullptr,
        .func = [](int argc, char** argv) -> int {
            int nerrors = arg_parse(argc, argv, (void**)&sk2_steps_args);
            if (nerrors != 0) {
                arg_print_errors(stderr, sk2_steps_args.end, argv[0]);
                return 1;
            }

            const char* str = sk2_steps_args.input->sval[0];
            size_t len = strlen(str);
            uint32_t parsed_val = 0;

            if (len == 1) {
                if (std::isxdigit(static_cast<unsigned char>(str[0]))) {
                    parsed_val = static_cast<uint32_t>(std::strtoul(str, nullptr, 16));
                } else {
                    printf("Error: Single-character input '%s' is not a valid hex digit (0-F).\n", str);
                    return 1;
                }
            } else if (len > 1) {
                for (size_t i = 0; i < len; ++i) {
                    if (!std::isdigit(static_cast<unsigned char>(str[i]))) {
                        printf("Error: Invalid input '%s'. Multi-character inputs must be decimal integers.\n", str);
                        return 1;
                    }
                }
                parsed_val = static_cast<uint32_t>(std::strtoul(str, nullptr, 10));
            } else {
                printf("Error: Empty input.\n");
                return 1;
            }
            
            s_stepper_ref->set_sk2_steps_async(parsed_val);
            printf("Command queued: Set SK2 value -> %lu (0x%X)\n", parsed_val, (unsigned int)parsed_val);
            return 0;
        },
        .argtable = &sk2_steps_args,
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