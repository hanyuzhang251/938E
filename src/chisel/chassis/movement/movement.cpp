#include "movement.h"

namespace chisel {

Movement::Movement(Pose* pose, const uint32_t life, const bool async) : curr_pose(pose), life(life), async(async) {}

std::pair<float, float> Movement::get_controls() {
    return controls;
}

TurnToHeading::TurnToHeading(Pose* pose, const float target_heading, const uint32_t life, const bool async)
    : Movement(pose, life, async), target_heading(target_heading) {}

void TurnToHeading::update() {
    angular_pid_control = deg_err(target_heading, curr_pose->h);
}

void TurnToHeading::push_controls() {
    controls.first = 0;
    controls.second = angular_pid_control;
}

TurnToPoint::TurnToPoint(Pose* pose, const Pose& target_point, const uint32_t life, const bool async)
    : Movement(pose, life, async), target_point(target_point) {}

void TurnToPoint::update() {
    angular_pid_control = deg_to_point(target_point);
}

void TurnToPoint::push_controls() {
    controls.first = 0;
    controls.second = angular_pid_control;
}

}