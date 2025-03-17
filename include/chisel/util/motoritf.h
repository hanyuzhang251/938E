#pragma once

#include "main.h"

#include <queue>

namespace chisel {

constexpr int MOTOR_COAST = 1001;
constexpr int MOTOR_BRAKE = 1002;

struct Command {
    float power;
    int priority;
    uint32_t life;

    void dismiss();

    Command(float power, int priority, uint32_t life = 1000 * 60 * 20);
};

struct MotorItf {
    pros::Motor* motor;

    auto comp = [](const Command* a, const Command* b){return a->priority < b->priority;};
    std::priority_queue<Command*, std::vector<Command*>, std::decltype(comp)> command_queue;

    explicit MotorItf(pros::Motor* motor);

    void assign_command(Command* command);

    void update();

    void push();
};

} // namespace chisel