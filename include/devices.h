#pragma once

#include "chisel/util/motoritf.h"
#include "main.hpp"
#include "ports.h"

inline pros::MotorGroup left_motors ({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
inline pros::MotorGroup right_motors ({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});
inline pros::Motor intake (INTAKE_PORT);
inline auto intake_itf = chisel::MotorItf(&intake);
inline pros::Motor arm (ARM_PORT);
inline pros::Imu imu (IMU_PORT);
inline pros::Optical optical (OPTICAL_PORT);
inline chisel::DriveTrain drive_train (
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    15, // track width
    450 // wheel rpm
);
inline chisel::DriveSettings lateral_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};
inline chisel::DriveSettings angular_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};
inline chisel::Odom odom {
    {0, 0, 0},
    {0, 0, 0},
    &imu,
    &drive_train,
    nullptr,
    0
};
inline chisel::PIDSettings angular_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

inline chisel::PIDSettings lateral_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};
inline chisel::Chassis chassis = {&drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_settings, &lateral_pid_settings, true};
inline pros::Controller master(pros::E_CONTROLLER_MASTER);