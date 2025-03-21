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
    while (chassis->enabled.load()) {
        chassis->update();
        wait(PROCESS_DELAY);
    }
}

Chassis::Chassis(DriveTrain* drive_train, DriveSettings *drive_settings, Odom* odom, PIDSettings* angular_pid_settings, PIDSettings* lateral_pid_settings, const bool enabled_):
    drive_train(drive_train), drive_settings(drive_settings), odom(odom), angular_pid_settings(angular_pid_settings), lateral_pid_settings(lateral_pid_settings), enabled(enabled_) {
    update_task = pros::Task(chassis_update, this);
}

} // namespace chisel