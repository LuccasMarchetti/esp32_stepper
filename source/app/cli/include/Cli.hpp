#pragma once

#include "StepperTask.hpp"
#include "esp_console.h"

/**
 * @brief Interactive Serial Command Line Interface (CLI).
 * 
 * Manages UART REPL commands for system tasks (StepperTask and future BLDC tasks).
 */
class Cli {
public:
    Cli() = default;
    ~Cli();

    /**
     * @brief Initializes the console interface and registers CLI commands.
     * @param stepper_task Reference to the active StepperTask instance.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init(StepperTask& stepper_task);

    /**
     * @brief Starts the interactive REPL prompt task.
     * @return true if REPL started successfully.
     */
    bool start();

private:
    void register_stepper_commands();

    StepperTask* stepper_task_ = nullptr;
    esp_console_repl_t* repl_ = nullptr;
    bool is_initialized_ = false;
};