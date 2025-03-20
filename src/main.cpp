#include "main.hpp"

#include "chisel/util/motoritf.h"

pros::MotorGroup left_motors ({1, 2, 3});
pros::MotorGroup right_motors ({4, 5, 6});

pros::Motor intake (7);
auto intake_itf = chisel::MotorItf(&intake);

pros::Motor arm (8);

pros::Imu imu (21);

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
