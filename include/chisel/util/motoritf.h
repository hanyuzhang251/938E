#pragma once

#include "main.h"

#include <queue>
#include <vector>

namespace chisel {
    struct Command {
        int32_t power;
        int32_t priority;
        uint32_t life;

        void dismiss();

        Command(int32_t power, int32_t priority, uint32_t life = 1000 * 60 * 20);
    };

    struct MotorItf {
        pros::Motor* motor;

        int32_t final_power = 0;
        pros::motor_brake_mode_e_t final_motor_brake_mode = pros::E_MOTOR_BRAKE_HOLD;

        struct compare_command {
            bool operator()(const Command* a, const Command* b) const {
                return a->priority < b->priority;
            }
        };

        std::priority_queue<Command*, std::vector<Command*>, compare_command> command_queue;

        explicit MotorItf(pros::Motor* motor);

        void assign_command(Command* command);

        int clean_commands();

        void update();

        void push_update() const;
    };

} // namespace chisel