#include "chisel/chassis/chassis.h"

#include "robot_config.h"

namespace chisel {

int Chassis::clean_commands() {
    if (motion_queue.empty()) return 0;

    int clear_count = 0;

    while (!motion_queue.empty()) {
        if (const auto top_comand = motion_queue.front(); top_comand->life <= 0) {
            motion_queue.pop();
            ++clear_count;
        } else {
            break;
        }
    }
    return clear_count;
}

void Chassis::update_motions() const {
    printf("\nUPDATE MOTIONS CALLED\n");

    if (motion_queue.empty()) {
        printf("\tno motions available, returning\n");
        return;
    }

    const auto top_motion = motion_queue.front();
    top_motion->life--;

    printf("\ttop motion life=%ld\n", top_motion->life);

    printf("\ttop motion pose=(%f, %f, %f)\n", top_motion->curr_pose->x.load(), top_motion->curr_pose->y.load(), top_motion->curr_pose->h.load());

    top_motion->update();
    top_motion->push_controls();

    printf("\ttop motion raw controls: lat=%f, ang=%f\n", top_motion->get_controls().first, top_motion->get_controls().second);

    if (state.load() == AUTON_STATE) {
        lateral_pid_controller->target.store(top_motion->controls.first);

        printf("\tUPDATING LATERAL PID...\n");
        pid_handle_process(*lateral_pid_controller);
        printf("\tUPDATING ANGULAR PID...\n");
        pid_handle_process(*angular_pid_controller);

        printf("\tlat-o=%f, ang-o=%f\n", lateral_pid_controller->output.load(), angular_pid_controller->output.load());

        float left_power = lateral_pid_output.load() + angular_pid_output.load();
        float right_power = lateral_pid_output.load() - angular_pid_output.load();

        printf("\tleft-p=%f, right-p=%f\n", left_power, right_power);

        left_power = abs_clamp(left_power, top_motion->min_speed, top_motion->max_speed);
        right_power = abs_clamp(right_power, top_motion->min_speed, top_motion->max_speed);

        (void)drive_train->left_motors->move(left_power);
        (void)drive_train->left_motors->move(right_power);
    }
}

void Chassis::update() {
    odom->predict_with_ime();
    // don't consider odom cause we don't have it yet
    odom->push_prediction(true, false);
    odom->load_pose();

    clean_commands();
    update_motions();
}

static void chassis_update(void* param) {
    auto *chassis = static_cast<Chassis*>(param);
    while(true) {
        if (chassis->enabled.load()) {
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