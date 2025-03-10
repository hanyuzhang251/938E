#pragma once

#include "main.h"
#include "chisel/util.h"
#include "chisel/config.h"
#include "chisel/data/pose.h"
#include <vector>

namespace chisel {

Pose solve_imu_bias(int32_t timeout);

struct TrackingWheel {
    pros::Rotation *rotation_sensor;
    Pose offset;
    float wheel_size;

    TrackingWheel(
        pros::Rotation* rotation_sensor, Pose offset,
        float wheel_size
    );
};

struct TrackingSetup {
    std::vector<TrackingWheel> tracking_wheel_list;

    TrackingSetup(
        TrackingWheel* tracking_wheel_list_ptr,
        int tracking_wheel_count
    );
};

} // namespace chisel