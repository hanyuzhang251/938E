#pragma once

#include "main.h"
#include "chisel/chisel.h"

constexpr pros::digi_button MENU_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_Y;
inline chisel::Toggle menu_toggle(false);
constexpr pros::digi_button MENU_SELECT_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
inline chisel::Toggle menu_select_toggle(false);
constexpr pros::digi_button MENU_BACK_BUTTON = pros::E_CONTROLLER_DIGITAL_B;
inline chisel::Toggle menu_back_toggle(false);

constexpr pros::digi_button MENU_POINTER_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_DOWN;
inline chisel::Toggle menu_pointer_down_toggle(false);
constexpr pros::digi_button MENU_POINTER_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
inline chisel::Toggle menu_pointer_up_toggle(false);

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::anlg_button TURN_JOYSTICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;

constexpr pros::digi_button LDOINKER_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_LEFT;
inline chisel::Toggle ldoinker_toggle(false);

constexpr pros::digi_button RDOINKER_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_RIGHT;
inline chisel::Toggle rdoinker_toggle(false);

constexpr pros::digi_button MOGO_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
inline chisel::Toggle mogo_toggle(false);

constexpr pros::digi_button ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;

constexpr pros::digi_button ARM_MACRO_CYCLE_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
inline chisel::Toggle arm_macro_cycle_toggle(false);

constexpr int32_t DT_FL_PORT = -6;
constexpr int32_t DT_ML_PORT = -14;
constexpr int32_t DT_BL_PORT = -15;

constexpr int32_t DT_FR_PORT = 3;//#####
constexpr int32_t DT_MR_PORT = 4;//#####
constexpr int32_t DT_BR_PORT = 5;//#####

constexpr int32_t INTAKE_PORT = -2;//#####
constexpr int32_t ARM_PORT = -9;
constexpr int32_t MOGO_PORT = 1;
constexpr int32_t LDOINKER_PORT = 2;
constexpr int32_t RDOINKER_PORT = 3;

constexpr int32_t IMU_PORT = 16;
constexpr int32_t OPTICAL_PORT = 11;

inline pros::MotorGroup left_motors({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
inline pros::MotorGroup right_motors({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

inline pros::Motor intake(INTAKE_PORT);
inline auto intake_itf = chisel::MotorItf(&intake);
inline chisel::Command driver_intake_command = {0, 326};
inline chisel::Command auton_intake_command = {0, 551};
inline chisel::Command unstuck_intake_command = {0, 0};
inline chisel::Command color_sort_command = {0, 0};

constexpr double BLUE_RING_HUE = 215;
constexpr double RED_RING_HUE = 5;
constexpr double RING_HUE_TOLOERANCE = 15;

constexpr uint32_t COLOR_SORT_COOLDOWN = 500;

constexpr float ARM_SPEED = 240;

constexpr float ARM_LOW_POS = 250;
constexpr float MAX_ARM_POS = 9968;
constexpr float ARM_LOAD_POS = 190;
constexpr float ARM_SCORE_POS = 690;
constexpr float ARM_ALLIANCE_POS = 920;

inline int arm_macro_cycle_index = 0;

inline bool arm_clamp = true;

inline pros::Motor arm(ARM_PORT);

inline chisel::PIDSetting arm_pid_settings{
    0.7, 0, 1, 10, 999, 999, 999, 0, 999
};
inline std::atomic<float> arm_pos(0);
inline std::atomic<float> arm_target_pos(0);
inline std::atomic<float> arm_pid_output(0);
inline chisel::PIDController arm_pid_controller = {
    arm_pos,
    arm_target_pos,
    arm_pid_output,
    arm_pid_settings,
    0,
    127,
    1000 * 60 * 20,
    [](const float target, const float value) {
        const float error = target - value;

        if (!arm_clamp || target > ARM_LOW_POS || value > ARM_LOW_POS || value < target) {
            return error;
        }

        return error / (2 * (ARM_LOW_POS / value));
    }
};

inline void reset_arm() {
    (void)arm.set_brake_mode(pros::MotorBrake::coast);
    (void)arm.tare_position();
    arm_target_pos.store(0);

    for (int i = 0; i < 500; ++i) {
        (void)arm.move(0);
        chisel::wait(1);
    }

    arm_target_pos.store(0);
    (void)arm.tare_position();
    (void)arm.set_brake_mode(pros::MotorBrake::hold);
}

inline pros::adi::DigitalOut mogo(MOGO_PORT);
inline pros::adi::DigitalOut ldoinker(LDOINKER_PORT);
inline pros::adi::DigitalOut rdoinker(RDOINKER_PORT);

inline pros::Imu imu(IMU_PORT);
inline pros::Optical optical(OPTICAL_PORT);

inline chisel::DriveTrain drive_train(
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    9.5, // track width
    3.0f/4.0f // ratio
);

inline chisel::DriveSettings lateral_drive_settings{
    3,
    10,
    chisel::LINEAR_CURVE
};
inline chisel::DriveSettings angular_drive_settings{
    3,
    10,
    chisel::LINEAR_CURVE
};

inline chisel::Odom odom{
    {0, 0, 0},
    {0, 0, 0},
    &imu,
    &drive_train,
    nullptr,
    0
};

inline chisel::PIDSetting angular_pid_settings{
    2, // kp
    0.1, // ki
    39, // kd
    2, // tolerance
    30, // wind
    999, // clamp
    999, // slew
    1, // small error
    60 // large error
};
inline std::atomic<float> target_heading (0);
inline std::atomic<float> angular_pid_output (0);
inline chisel::PIDController angular_pid_controller {
    odom.pose.h,
    target_heading,
    angular_pid_output,
    angular_pid_settings,
    0,
    127,
    1000 * 60 * 20,
    chisel::deg_err
};

inline chisel::PIDSetting lateral_pid_settings{
    4.5, // kp
    0.1, // ki
    3, // kd
    2, // tolerance
    12, // wind
    999, // clamp
    999, // slew
    0, // small error
    999 // large error
};
inline std::atomic<float> prev_dist (0);
inline std::atomic<float> current_dist (0);
inline std::atomic<float> target_dist (0);
inline std::atomic<float> lateral_pid_output (0);
inline chisel::PIDController lateral_pid_controller {
    current_dist,
    target_dist,
    lateral_pid_output,
    lateral_pid_settings,
    0,
    127,
    1000 * 60 * 20,
    nullptr
};

inline chisel::Chassis chassis = {
    &drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_controller, &lateral_pid_controller,
    false
};

inline void device_init() {
    (void) left_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);
    (void) right_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);

    optical.set_integration_time(PROCESS_DELAY);
    optical.set_led_pwm(100);

    intake_itf.assign_command(&color_sort_command);
    intake_itf.assign_command(&unstuck_intake_command);

    target_dist.store(0);
    current_dist.store(0);
}

inline void auton_end() {
    chassis.state = DRIVE_STATE;
    wait(15);

    (void) left_motors.set_brake_mode_all(pros::MotorBrake::coast);
    (void) right_motors.set_brake_mode_all(pros::MotorBrake::coast);

    (void) left_motors.move(0);
    (void) right_motors.move(0);

    auton_intake_command.priority = 0;
    auton_intake_command.dismiss();
}

inline void auton_init(const float h_offset = 0, const float arm_offset = 0) {
    chassis.state = CRASHOUT;
    wait(15);

    intake_itf.assign_command(&auton_intake_command);

    odom.internal_pose.x.store(0);
    odom.internal_pose.y.store(0);
    odom.internal_pose.h.store(0);
    odom.pose.x.store(0);
    odom.pose.y.store(0);
    odom.pose.h.store(0);
    odom.pose_offset.x.store(0);
    odom.pose_offset.y.store(0);
    odom.pose_offset.h.store(h_offset);

    odom.prev_left_pos = 0;
    odom.prev_right_pos = 0;
    (void)left_motors.tare_position_all();
    (void)right_motors.tare_position_all();

    prev_dist.store(0);
    current_dist.store(0);

    (void)mogo.set_value(true);

    target_heading.store (odom.pose_offset.h);

    current_dist.store(0);
    target_dist.store(current_dist.load());

    (void)arm.tare_position();
    arm_pos.store(arm_offset);
    arm_target_pos.store(arm_pos.load());

    chassis.state = AUTON_STATE;
    wait(15);
}

inline void device_update() {
    intake_itf.clean_commands();
    intake_itf.update();
    intake_itf.push_control();

    arm_pos.store(arm.get_position());
    pid_handle_process(arm_pid_controller);
    (void)arm.move(arm_pid_output.load());

    if (chassis.state == AUTON_STATE) {
        pid_handle_process(angular_pid_controller);
        pid_handle_process(lateral_pid_controller);

        float left_power = lateral_pid_output.load() + angular_pid_output.load();
        float right_power = lateral_pid_output.load() - angular_pid_output.load();

        (void)drive_train.left_motors->move(left_power);
        (void)drive_train.right_motors->move(right_power);
    }
}