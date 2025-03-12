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

}


} // namespace chisel