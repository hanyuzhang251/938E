#include "odom.h"

#include "../../../include/robot_config.h"
#include <string>
#include <vector>

namespace chisel {

Pose solve_imu_bias(const pros::Imu& imu, const uint32_t timeout) {
    Pose bias (0, 0, 0);
    auto [bx, by, bh] = bias();

    const uint32_t end = pros::millis() + timeout;

    int ticks = 0;

    printf("%sstart solving imu bias\n", prefix().c_str());

    while (pros::millis() < end) {
        bx.fetch_add(imu.get_accel().x);
        by.fetch_add(imu.get_accel().y);

        ++ticks;

        wait(PROCESS_DELAY);
    }

    bx.store(bx.load() / ticks);
    by.store(by.load() / ticks);

    printf("%ssolved imu bias for %d ticks, result: bx=%f, by=%f\n", prefix().c_str(), ticks, bx.load(), by.load());

    return {bx.load(), by.load(), bh.load()};
}

TrackingWheel::TrackingWheel(
    pros::Rotation* rotation_sensor, const Pose& offset,
    const float wheel_size)
    : rotation_sensor(rotation_sensor), offset(offset),
    wheel_size(wheel_size) {
        printf("%screate new TrackingWheel: sensor(%d), offset=(%f, %f, %f), wheel_size=%f\n", prefix().c_str(), rotation_sensor->get_port(), offset.x.load(), offset.y.load(), offset.h.load(), wheel_size);
    };

Odom::Odom(
    const Pose &pose,
    const Pose &pose_offset,
    pros::Imu* imu,
    DriveTrain* drive_train,
    TrackingWheel* tracking_wheel_list_ptr,
    const int tracking_wheel_count,
    const int MaxImuInitAttempts
): pose(pose), pose_offset(pose_offset), imu(imu), drive_train(drive_train), MaxImuResetAttempts(MaxImuInitAttempts) {
    tracking_wheel_list.reserve(tracking_wheel_count);
    tracking_wheel_list.insert(tracking_wheel_list.end(),
        tracking_wheel_list_ptr, tracking_wheel_list_ptr + tracking_wheel_count);

    printf("%screate new Odom: ime=%s odom=%s\n", prefix().c_str(), (!drive_train) ? "yes" : "no", ((tracking_wheel_count > 0) ? std::to_string(tracking_wheel_count) : "no").c_str());
}

int32_t Odom::initialize_imu() {
    if (!imu) {
        printf("%snot using imu; skipping imu initialization\n", prefix().c_str());
        return 0;
    }

    printf("%sinitializing imu\n", prefix().c_str());
    for (int i = 1; i <= MaxImuResetAttempts; ++i) {
        if (imu->reset()) {
            printf("%ssuccessfully reset imu on attempt %d\n", prefix().c_str(), i);
            break;
        }
        if (i == MaxImuResetAttempts) {
            printf("%smu reset unsuccessful, continuing without imu\n", prefix().c_str());
            return -1;
        }
        printf("%smu reset unsuccessful, retrying...\n", prefix().c_str());
    }

    printf("%ssolving imu bias\n", prefix().c_str());
    imu_bias = solve_imu_bias(*imu, 2000);

    printf("%simu initialization complete\n", prefix().c_str());
    return 0;
}

void Odom::initialize() {
    printf("%sinitializing Odom\n", prefix().c_str());

    if (initialize_imu() == -1) {
        imu = nullptr;
    }

    printf("%sOdom initialization complete\n", prefix().c_str());
}


void Odom::predict_with_ime() {
    const double left_pos = drive_train->left_motors->get_position();
    const double right_pos = drive_train->right_motors->get_position();

    const double dist = ((left_pos - prev_left_pos) + (right_pos - prev_right_pos)) / 2 * drive_train->magic_number;

    prev_left_pos = left_pos;
    prev_right_pos = right_pos;

    current_dist.fetch_add(dist);

    auto [ipos_x, ipos_y, ipos_h] = ime_estimate();

    const double h_rads = ipos_h * M_PI / 180;
    ipos_x.fetch_add(std::cos(h_rads) * dist);
    ipos_y.fetch_add(std::sin(h_rads) * dist);

    ipos_h.store(imu->get_heading());
}

void Odom::push_prediction(bool consider_ime, bool consider_odom) {
    // might do some stuff with filtering for noise but we lwky dont even have odom yet.
    internal_pose = ime_estimate;
}

void Odom::load_pose() {
    pose = Pose::add(internal_pose, pose_offset);
}

} // namespace chisel