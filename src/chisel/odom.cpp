#include "odom.h"

namespace chisel {

Pose::Pose(float x_pos, float y_pos, float head) {
    x.store(x_pos);
    y.store(y_pos);
    h.store(head);
}

auto Pose::operator()() {
    return std::tie(x, y, h);
}

Pose solve_imu_bias(pros::Imu inertial, int32_t timeout) {
    Pose bias (0, 0, 0);
    auto [bx, by, bh] = bias();

    int32_t end = pros::millis() + timeout;

    int ticks = 0;

    while (pros::millis() < end) {
        bx.fetch_add(inertial.get_accel().x);
        by.fetch_add(inertial.get_accel().y);

        ++ticks;

        wait(PROCESS_DELAY);
    }

    bx.store(bx.load() / ticks);
    by.store(by.load() / ticks);

    return Pose(bx.load(), by.load(), bh.load());
}

} // namespace chisel