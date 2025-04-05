#pragma once

#include "main.h"

#include <queue>
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
     * @brief Uses a priority queue to allow motor controls based on priority of command.
     *
     * Brief could be much better, but I find it difficult to word it in a better way.
     * Uses a priority queue of commands sorted by their priority, and moves the motor to the power.
     */
    struct MotorItf {
        pros::Motor* motor;

        int32_t final_power = 0;

        /**
         * @brief Sorts commands by their priority.
         */
        struct compare_command {
            bool operator()(const Command* a, const Command* b) const {
                return a->priority < b->priority;
            }
        };

        std::priority_queue<Command*, std::vector<Command*>, compare_command> command_queue;

        /**
         * @brief MotorItf constructor.
         *
         * @param motor Pointer to the motor to control.
         */
        explicit MotorItf(pros::Motor* motor);

        /**
         * @param command Pointer to the command to be emplaced to the priority queue.
         */
        void assign_command(Command* command);


        /**
         * @brief Cleans the priority queue by removing commands that have died (life <= 0)
         *
         * @return The number of commands cleaned. Will often be zero.
         */
        int clean_commands();

        /**
         * @brief Gets the motor power from the top command. Also removes a tick of life from it.
         */
        void update();

        /**
         * @brief Moves the motor to the power (terrible wording).
         */
        void push_control() const;
    };

} // namespace chisel