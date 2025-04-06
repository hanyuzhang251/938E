#pragma once

#include "chisel/chassis/drive.h"
#include "chisel/odom/odom.h"
#include "chisel/pid.h"
#include "chisel/chassis/movement/movement.h"

#include <atomic>
#include <queue>

namespace chisel {

    struct Chassis {
        std::atomic<int> state = std::atomic(INIT_STATE);

        DriveTrain *drive_train;
        DriveSettings *lateral_drive_settings;
        DriveSettings *angular_drive_settings;

        Odom *odom;

        PIDController *angular_pid_controller;
        PIDController *lateral_pid_controller;

        std::queue<Motion*> motion_queue;

        std::atomic<bool> enabled;

        void update();

        Chassis(DriveTrain* drive_train, DriveSettings* lateral_drive_settings, DriveSettings* angular_drive_settings, Odom* odom, PIDController* angular_pid_controller, PIDController* lateral_pid_controller, bool enabled_ = false);

        void initialize() const;

    private:
        int clean_commands();

        void update_motions() const;
    };

} // namespace chisel