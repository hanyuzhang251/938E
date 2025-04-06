#pragma once

#include "main.h"

#include <vector>
#include "chisel/config.h"

namespace chisel {
    /**
     * @brief Stores the power, priority, and life of the command.
     */
    struct Command {
        int32_t power;
        int32_t priority;
        uint32_t life;

        /**
         * @brief Kills the command.
         *
         * Does this by setting its life to -1. The command will be removed from the MotorItf when it is cleaned.
         */
        void dismiss();

        /**
         * @brief Command constructor
         *
         * @param power The power to be sent to the motor.
         *        In the range [-127.0f, +127.0f] otherwise the MotorItf will reject the command.
         * @param priority Priority of the command. Higher priority commands override lower priority ones.
         * @param life Life of the command in ticks. Two hours by default.
         *
         * @note Note how both power and priority can be changed;
         *       each command should be long lasting and shouldn't be cleaned just because its power needs to change.
         */
        Command(int32_t power, int32_t priority, uint32_t life = 1000 * 60 * 20 / PROCESS_DELAY);
    };

    /**
     * @brief Interface for managing motor control using a prioritized command system.
     */
    struct MotorItf {
        pros::Motor *motor;

        int32_t final_power = 0;

        std::vector<Command *> command_list;
        Command* top_command = nullptr;

        /**
         * @brief MotorItf constructor.
         *
         * @param motor Pointer to the motor to control.
         */
        explicit MotorItf(pros::Motor *motor);

        /**
         * @param command Pointer to the command to be added.
         */
        void assign_command(Command *command);

        /**
         * @brief Cleans the comand list by removing commands with life <= 0
         */
        void clean_commands();

        /**
         * @brief Finds the top command and sets the final power. Also decrements life for allcommands.
         */
        void update();

        /**
         * @brief Sets the motor power to the final power.
         */
        void push_control() const;
    };
} // namespace chisel
