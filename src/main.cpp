#include "main.hpp"
#include "ports.h"

#include "chisel/util/motoritf.h"

pros::MotorGroup left_motors ({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
pros::MotorGroup right_motors ({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

pros::Motor intake (INTAKE_PORT);
auto intake_itf = chisel::MotorItf(&intake);

pros::Motor arm (ARM_PORT);

pros::Imu imu (IMU_PORT);

chisel::DriveTrain drive_train (
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    15, // track width
    450 // wheel rpm
);

chisel::DriveSettings drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};

chisel::Pose robot_pose = {0, 0, 0};
chisel::Pose robot_start_pose = {0, 0, 0};

chisel::Odom odom {
    robot_pose,
    robot_start_pose,
    &imu,
    &drive_train,
    nullptr,
    0
};

chisel::PIDSettings angular_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::PIDSettings lateral_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::Chassis chassis = {&drive_train, &drive_settings, &odom, &angular_pid_settings, &lateral_pid_settings, true};