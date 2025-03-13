#pragma once

#include "main.h"
#include "chisel/util/util.h"
#include "chisel/chassis/drive.h"
#include "chisel/data/pose.h"
#include "chisel/config.h"
#include <vector>

namespace chisel {

Pose solve_imu_bias(const pros::Imu& imu, uint32_t timeout);

struct TrackingWheel {
    pros::Rotation *rotation_sensor;
    Pose offset;
    float wheel_size;

    float prev_pos = 0;

    TrackingWheel(
        pros::Rotation* rotation_sensor, const Pose& offset,
        float wheel_size
    );
};

struct Odom {
    Pose pose;
    Pose internal_pose = Pose(0, 0, 0);
    Pose pose_offset;

    Pose ime_estimate = Pose(0, 0, 0);
    Pose odom_estimate = Pose(0, 0, 0);

    pros::Imu *imu;
    DriveTrain *drive_train;
    std::vector<TrackingWheel> tracking_wheel_list;

    float prev_left_pos = 0;
    float prev_right_pos = 0;

    Odom(
        const Pose &pose,
        const Pose &pose_offset,
        pros::Imu* imu,
        DriveTrain* drive_train,
        TrackingWheel* tracking_wheel_list_ptr,
        int tracking_wheel_count
    );

    void predict_with_ime();

    void predict_with_odom();

    void push_prediction(bool consider_ime = true, bool consider_odom = true);

    void load_pose();
};

} // namespace chisel