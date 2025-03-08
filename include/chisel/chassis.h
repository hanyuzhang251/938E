#pragma once

#include "main.h"
#include "odom.h"

namespace chisel {

struct Chassis {
    pros::MotorGroup &left_drive;
    pros::MotorGroup &right_drive;
    float drive_wheel_size;
    float drive_ratio;
    float track_width;

    pros::Rotation &v_odom;
    pros::Rotation &h_odom;
    Pose v_odom_offset;
    Pose h_odom_offset;
    float v_odom_wheel_size;
    float h_odom_wheel_size;

    pros::Imu &inertial;

    Chassis(pros::MotorGroup &left, pros::MotorGroup &right, float wheel_size, float ratio, float width,
        pros::Rotation &vOdom, pros::Rotation &hOdom, Pose vOffset, Pose hOffset,
        float vWheelSize, float hWheelSize, pros::Imu &imu)
    : left_drive(left), right_drive(right), drive_wheel_size(wheel_size), drive_ratio(ratio), track_width(width),
      v_odom(vOdom), h_odom(hOdom), v_odom_offset(vOffset), h_odom_offset(hOffset),
      v_odom_wheel_size(vWheelSize), h_odom_wheel_size(hWheelSize), inertial(imu) {}
};

} // namespace chisel