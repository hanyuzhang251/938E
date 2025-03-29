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

    PIDSettings *angular_pid_settings;
    PIDSettings *lateral_pid_settings;

    std::queue<std::unique_ptr<Movement>> instruction_queue;

    std::atomic<bool> enabled;

    void update() const;

    Chassis(DriveTrain* drive_train, DriveSettings* lateral_drive_settings, DriveSettings* angular_drive_settings, Odom* odom, PIDSettings* angular_pid_settings, PIDSettings* lateral_pid_settings, bool enabled_ = true);

    void initialize() const;
};

} // namespace chisel