#pragma once

#include "main.h"
#include "chisel/util.h"
#include "chisel/chassis/drive.h"
#include "chisel/data/pose.h"
#include "chisel/config.h"
#include <vector>

namespace chisel {

Pose solve_imu_bias(int32_t timeout);

struct TrackingWheel {
    pros::Rotation *rotation_sensor;
    Pose offset;
    float wheel_size;

    TrackingWheel(
        pros::Rotation* rotation_sensor, const Pose& offset,
        float wheel_size
    );
};

struct Odom {
    Pose pose;
    Pose internal_pose;
    Pose pose_offset;

    DriveTrain *drive_train;
    std::vector<TrackingWheel> tracking_wheel_list;

    Odom(
        const Pose &pose,
        const Pose &internal_pose,
        const Pose &pose_offset,
        DriveTrain* drive_train,
        TrackingWheel* tracking_wheel_list_ptr,
        int tracking_wheel_count
    );

    void update_with_ime();

    void update_with_odom();

    void update();
};

} // namespace chisel