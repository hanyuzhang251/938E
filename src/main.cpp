#include "main.hpp"

#include "chisel/util/motoritf.h"

pros::MotorGroup left_motors ({1, 2, 3});
pros::MotorGroup right_motors ({4, 5, 6});

pros::Motor intake (7);
auto intake_itf = chisel::MotorItf(&intake);

pros::Motor arm (8);

chisel::DriveTrain drive_train (
    &left_motors,
    &right_motors,
    3.25f, // wheel diameter
    15, // track width
    480 // wheel rpm
);