#include "chisel/chassis/chassis.h"

namespace chisel {

void Chassis::update() const {
    odom->predict_with_ime();
    // don't consider odom cause we don't have it yet
    odom->push_prediction(true, false);
    odom->load_pose();
}

static void chassis_update(void* param) {
    const auto *chassis = static_cast<Chassis*>(param);
    for (int i = 0; i < INT_MAX; ++i) {
        chassis->update();
        wait(PROCESS_DELAY);
    }
}

Chassis::Chassis(DriveTrain* drive_train, DriveSettings *lateral_drive_settings, DriveSettings *angular_drive_settings, Odom* odom, PIDSettings* angular_pid_settings, PIDSettings* lateral_pid_settings, const bool enabled_):
    drive_train(drive_train), lateral_drive_settings(lateral_drive_settings), angular_drive_settings(angular_drive_settings), odom(odom), angular_pid_settings(angular_pid_settings), lateral_pid_settings(lateral_pid_settings), enabled(enabled_) {
    auto update_task = pros::Task(chassis_update, this);
}

void Chassis::initialize() const {
    printf("%sinitializing chassis\n", prefix().c_str());

    odom->initialize();
}

} // namespace chisel