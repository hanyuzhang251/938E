#include "chisel/chassis/chassis.h"

namespace chisel {

void Chassis::update() const {
    odom->predict_with_ime();
    // don't consider odom cause we don't have it yet
    odom->push_prediction(true, false);
    odom->load_pose();
}

static void generic_chassis_update(void* param) {
    const auto *chassis = static_cast<Chassis*>(param);
    while (chassis->enabled.load()) {
        chassis->update();
        wait(PROCESS_DELAY);
    }
}

Chassis::Chassis(DriveTrain* drive_train, Odom* odom, const bool enabled_): drive_train(drive_train), odom(odom), enabled(enabled_) {
    update_task = pros::Task(generic_chassis_update, this);
}

} // namespace chisel