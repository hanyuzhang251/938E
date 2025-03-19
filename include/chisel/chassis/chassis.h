#pragma once

#include "main.h"
#include "chisel/chassis/drive.h"
#include "chisel/odom/odom.h"
#include "chisel/pid.h"
#include "chisel/chassis/movement/movement.h"

#include <atomic>
#include <queue>

namespace chisel {

struct Chassis {
    DriveTrain *drive_train;
    Odom *odom;

    PIDSettings *angular_pid_settings;
    PIDSettings *lateral_pid_settings;

    std::queue<Movement> instruction_queue;

    pros::Task update_task = nullptr;
    std::atomic<bool> enabled;

    void update() const;

    Chassis(DriveTrain* drive_train, Odom* odom, PIDSettings* angular_pid_settings, PIDSettings* lateral_pid_settings, bool enabled_ = true);
};

} // namespace chisel