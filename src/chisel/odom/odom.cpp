#include "odom.h"

#include <vector>
#include <string>

namespace chisel {

Pose solve_imu_bias(const pros::Imu& inertial, const int32_t timeout) {
    Pose bias (0, 0, 0);
    auto [bx, by, bh] = bias();

    const int32_t end = pros::millis() + timeout;

    int ticks = 0;

    printf("%sstart solving imu bias\n", prefix().c_str());

    while (pros::millis() < end) {
        bx.fetch_add(inertial.get_accel().x);
        by.fetch_add(inertial.get_accel().y);

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
    const Pose &internal_pose,
    const Pose &pose_offset,
    DriveTrain* drive_train,
    TrackingWheel* tracking_wheel_list_ptr,
const int tracking_wheel_count
): pose(pose), internal_pose(internal_pose), pose_offset(pose_offset), drive_train(drive_train) {
    tracking_wheel_list.reserve(tracking_wheel_count);
    tracking_wheel_list.insert(tracking_wheel_list.end(),
        tracking_wheel_list_ptr, tracking_wheel_list_ptr + tracking_wheel_count);

    printf("%screate new Odom: ime=%s odom=%s\n", prefix().c_str(), (!drive_train) ? "yes" : "no", ((tracking_wheel_count > 0) ? std::to_string(tracking_wheel_count) : "no").c_str());
}

void Odom::update_with_ime() {
    const double left_pos = drive_train->left_motors->get_position();
    const double right_pos = drive_train->right_motors->get_position();

    const double dist = ((left_pos - prev_left_pos) + (right_pos - prev_right_pos)) / 2;

    auto [ipos_x, ipos_y, ipos_h] = internal_pose();

    const double h_rads = ipos_h * M_PI / 180;
    ipos_x.fetch_add(std::cos(h_rads) * dist);
    ipos_y.fetch_add(std::sin(h_rads) * dist);

    auto [mpos_x, mpos_y, mpos_h] = pose_offset();
    auto [pos_x, pos_y, pos_h] = pose();

    pos_x.store(mpos_x.load() + ipos_x.load());
    pos_y.store(mpos_y.load() + ipos_y.load());
    pos_h.store(mpos_h.load() + ipos_h.load());
}


} // namespace chisel