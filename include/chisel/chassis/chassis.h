#pragma once

#include "main.h"
#include "chisel/data/pose.h"

namespace chisel {

struct DriveTrain {
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;

    float wheel_size;
    float track_width;
    float rpm;

    DriveTrain(
        pros::MotorGroup *left_motors, pros::MotorGroup *right_motors,
        float wheel_size, float track_width, float rpm
    );
};


struct Chassis {
    DriveTrain *drive_train;

};

} // namespace chisel