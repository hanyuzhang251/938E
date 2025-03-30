#include "chisel/chassis/chassis.h"

namespace chisel {

void Chassis::handle_instructions() {
    auto top_movement = instruction_queue.front();

    if (top_movement->life <= 0) {
        instruction_queue.pop();
    }

    top_movement->update();
    top_movement->push_controls();
}


void Chassis::update() {
    odom->predict_with_ime();
    // don't consider odom cause we don't have it yet
    odom->push_prediction(true, false);
    odom->load_pose();

    if (!instruction_queue.empty()) handle_instructions();
}

static void chassis_update(void* param) {
    auto *chassis = static_cast<Chassis*>(param);
    for (int i = 0; i < INT_MAX; ++i) {
        chassis->update();
        wait(PROCESS_DELAY);
    }
}

Chassis::Chassis(DriveTrain* drive_train, DriveSettings *lateral_drive_settings, DriveSettings *angular_drive_settings, Odom* odom, chisel::PIDController* angular_pid_controller, PIDController* lateral_pid_controller, const bool enabled_):
    drive_train(drive_train), lateral_drive_settings(lateral_drive_settings), angular_drive_settings(angular_drive_settings), odom(odom), angular_pid_controller(angular_pid_controller), lateral_pid_controller(lateral_pid_controller), enabled(enabled_) {
    auto update_task = pros::Task(chassis_update, this);
}

void Chassis::initialize() const {
    printf("%sinitializing chassis\n", prefix().c_str());

    odom->initialize();
}

} // namespace chisel