#pragma once

#include "main.h"
#include "chisel/chassis/drive.h"
#include "chisel/data/pose.h"

namespace chisel {

struct Chassis {
    DriveTrain *drive_train;

};

} // namespace chisel