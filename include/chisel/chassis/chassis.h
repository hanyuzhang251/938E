#pragma once

#include "main.h"
#include "chisel/chassis/drive.h"
#include "chisel/odom/odom.h"
#include "chisel/chassis/movement/movement.h"

#include <queue>

namespace chisel {

struct Chassis {
    DriveTrain *drive_train;
    Odom *odom;

    std::queue<Movement> instruction_queue;
};

} // namespace chisel