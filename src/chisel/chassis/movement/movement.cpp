#include "movement.h"

namespace chisel {

ExitCondition::ExitCondition(const float range, const uint32_t time) : range(range), time(time) {
    reset();
}

bool ExitCondition::get_exit() const {
    return exit;
}

void ExitCondition::update(const float error) {
    if (error > range) start_time = pros::millis();

    if (pros::millis() - start_time >= time) exit = true;
}

void ExitCondition::reset() {
    start_time = UINT32_MAX;
    exit = false;
}

Motion::Motion(Pose* pose, const uint32_t life, const bool async, const float min_speed, const float max_speed)
    : curr_pose(pose), life(life), async(async), min_speed(min_speed), max_speed(max_speed) {}

std::pair<float, float> Motion::get_controls() {
    return controls;
}

TurnToHeading::TurnToHeading(Pose* pose, const float target_heading, const uint32_t life, const bool async)
    : Motion(pose, life, async), target_heading(target_heading) {}

void TurnToHeading::update() {
    angular_pid_control = deg_err(target_heading, curr_pose->h);
}

void TurnToHeading::push_controls() {
    controls.first = 0;
    controls.second = angular_pid_control;
}

TurnToPoint::TurnToPoint(Pose* pose, const Pose& target_point, const uint32_t life, const bool async)
    : Motion(pose, life, async), target_point(target_point) {}

void TurnToPoint::update() {
    angular_pid_control = deg_to_point(target_point);
}

void TurnToPoint::push_controls() {
    controls.first = 0;
    controls.second = angular_pid_control;
}

MoveToPoint::MoveToPoint(Pose* pose, const Pose& target_point, const uint32_t life, const bool async)
    : Motion(pose, life, async), target_point(target_point) {}

void MoveToPoint::update() {
    const Pose relative_target = Pose::sub(target_point, *curr_pose);

    const float target_heading = deg_to_point(relative_target);
    const float dist_to_travel = dist_to_point(relative_target);

    const float heading_proximity = 1 - std::abs(deg_err(curr_pose->h, target_heading)) / 90;

    pid_controls.first = dist_to_travel * heading_proximity;
    pid_controls.second = deg_err(curr_pose->h, target_heading);
}

void MoveToPoint::push_controls() {
    controls.first = pid_controls.first;
    controls.second = pid_controls.second;
}


}