#include "chisel/chassis/chassis.h"

#include "robot_config.h"

namespace chisel {

int Chassis::clean_commands() {
    if (command_queue.empty()) return 0;

    int clear_count = 0;

    while (!command_queue.empty()) {
        if (const auto top_comand = command_queue.front(); top_comand->life <= 0) {
            command_queue.pop();
            ++clear_count;
        } else {
            break;
        }
    }
    return clear_count;
}

void Chassis::update_commands() const {
    if (command_queue.empty()) return;

    const auto top_movement = command_queue.front();

    top_movement->update();
    top_movement->push_controls();

    if (state.load() == AUTON_STATE) {
        lateral_pid_controller->target.store(top_movement->controls.first);
        angular_pid_controller->target.store(top_movement->controls.second);

        (void)drive_train->left_motors->move(lateral_pid_output.load() + angular_pid_output.load());
        (void)drive_train->left_motors->move(lateral_pid_output.load() - angular_pid_output.load());
    }
}

void Chassis::update() {
    printf("%sCHASSIS UPDATE CALLED\n", prefix().c_str());
    odom->predict_with_ime();
    // don't consider odom cause we don't have it yet
    odom->push_prediction(true, false);
    odom->load_pose();

    clean_commands();
    update_commands();
}

static void chassis_update(void* param) {
    auto *chassis = static_cast<Chassis*>(param);
    while(true) {
        if (chassis->enabled.load()) {
            printf("%sCHASSIS ENABLED (%d), CALLING CHASSIS UPDATE\n", prefix().c_str(), chassis->enabled.load());
            chassis->update();
        }
        wait(PROCESS_DELAY);
    }
}

Chassis::Chassis(DriveTrain* drive_train, DriveSettings *lateral_drive_settings, DriveSettings *angular_drive_settings, Odom* odom, chisel::PIDController* angular_pid_controller, PIDController* lateral_pid_controller, const bool enabled_):
    drive_train(drive_train), lateral_drive_settings(lateral_drive_settings), angular_drive_settings(angular_drive_settings), odom(odom), angular_pid_controller(angular_pid_controller), lateral_pid_controller(lateral_pid_controller), enabled(enabled_) {
    pros::Task(chassis_update, this);
}

void Chassis::initialize() const {
    printf("%sinitializing chassis\n", prefix().c_str());

    odom->initialize();
}

} // namespace chisel