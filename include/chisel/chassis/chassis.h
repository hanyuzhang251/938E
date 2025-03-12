#pragma once

#include "main.h"
#include "chisel/chassis/drive.h"
#include "chisel/odom/odom.h"

namespace chisel {

struct Chassis {
    DriveTrain *drive_train;
    Odom *odom;
};

} // namespace chisel