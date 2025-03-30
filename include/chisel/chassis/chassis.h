#pragma once

#include "main.h"
#include "chisel/chassis/drive.h"
#include "chisel/odom/odom.h"
#include "chisel/pid.h"
#include "chisel/chassis/movement/movement.h"

#include <atomic>
#include <queue>
#include <memory>

namespace chisel {

struct Chassis {
    DriveTrain *drive_train;
    DriveSettings *lateral_drive_settings;
    DriveSettings *angular_drive_settings;

    Odom *odom;

    PIDController *angular_pid_controller;
    PIDController *lateral_pid_controller;

    std::queue<Movement*> instruction_queue;

    std::atomic<bool> enabled;

    void update();

    Chassis(DriveTrain* drive_train, DriveSettings* lateral_drive_settings, DriveSettings* angular_drive_settings, Odom* odom, PIDController* angular_pid_controller, PIDController* lateral_pid_controller, bool enabled_ = true);

    void initialize() const;

private:
    void handle_instructions();
};

} // namespace chisel