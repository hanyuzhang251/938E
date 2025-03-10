#include "odom.h"

#include <vector>

namespace chisel {

Pose solve_imu_bias(pros::Imu inertial, int32_t timeout) {
    Pose bias (0, 0, 0);
    auto [bx, by, bh] = bias();

    int32_t end = pros::millis() + timeout;

    int ticks = 0;

    printf("%sstart solving imu bias\n", prefix());

    while (pros::millis() < end) {
        bx.fetch_add(inertial.get_accel().x);
        by.fetch_add(inertial.get_accel().y);

        ++ticks;

        wait(PROCESS_DELAY);
    }

    bx.store(bx.load() / ticks);
    by.store(by.load() / ticks);

    printf("%ssolved imu bias for %d ticks, result: bx=%f, by=%f\n", prefix(), ticks, bx.load(), by.load());

    return Pose(bx.load(), by.load(), bh.load());
}

TrackingWheel::TrackingWheel(
    pros::Rotation* rotation_sensor, Pose offset,
    float wheel_size)
    : rotation_sensor(rotation_sensor), offset(offset),
    wheel_size(wheel_size) {
        printf("%screate new TrackingWheel: sensor(%d), offset=(%f, %f, %f), wheel_size=%f\n", prefix(), rotation_sensor->get_port(), offset.x.load(), offset.y.load(), offset.h.load(), wheel_size);
    };

TrackingSetup::TrackingSetup(
    TrackingWheel* tracking_wheel_list_ptr,
    int tracking_wheel_count
) {
    tracking_wheel_list.reserve(tracking_wheel_count);
    tracking_wheel_list.insert(tracking_wheel_list.end(),
        tracking_wheel_list_ptr, tracking_wheel_list_ptr + tracking_wheel_count);

    printf("%screate new TrackingSetup: tracking_wheel_count=%d\n", prefix(), tracking_wheel_count);
}

} // namespace chisel