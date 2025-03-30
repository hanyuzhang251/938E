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

constexpr pros::digi_button DOINKER_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
inline chisel::Toggle doinker_toggle(false);

constexpr pros::digi_button MOGO_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
inline chisel::Toggle mogo_toggle(false);

constexpr pros::digi_button ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;

constexpr pros::digi_button ARM_MACRO_CYCLE_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
inline chisel::Toggle arm_macro_cycle_toggle(false);

constexpr int32_t DT_FL_PORT = -6;
constexpr int32_t DT_ML_PORT = -7;
constexpr int32_t DT_BL_PORT = -8;

constexpr int32_t DT_FR_PORT = 3;
constexpr int32_t DT_MR_PORT = 4;
constexpr int32_t DT_BR_PORT = 5;

constexpr int32_t INTAKE_PORT = -2;
constexpr int32_t ARM_PORT = -9;
constexpr int32_t MOGO_PORT = 21;
constexpr int32_t DOINKER_PORT = 21;

constexpr int32_t IMU_PORT = 21;
constexpr int32_t OPTICAL_PORT = 21;

inline pros::MotorGroup left_motors({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
inline pros::MotorGroup right_motors({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

inline pros::Motor intake(INTAKE_PORT);
inline auto intake_itf = chisel::MotorItf(&intake);
inline chisel::Command driver_intake_command = {0, 326};
inline chisel::Command auton_intake_command = {0, 551};

constexpr float ARM_SPEED = 240;

constexpr float MAX_ARM_POS = 1800;
constexpr float ARM_LOAD_POS = 250;
constexpr float ARM_SCORE_POS = 800;

inline int arm_macro_cycle_index = 0;

inline pros::Motor arm(ARM_PORT);

inline chisel::PIDSettings arm_pid_settings{
    0.65, 0.05, 5, 100, 999, 999, 20, 250, 0
};
inline std::atomic<float> arm_pos(0);
inline std::atomic<float> arm_target_pos(0);
inline std::atomic<float> arm_pid_output(0);
inline chisel::PIDController arm_pid = {
    arm_pos,
    arm_target_pos,
    arm_pid_output,
    arm_pid_settings,
    127,
    0,
    1000 * 60 * 20,
    nullptr
};

inline pros::adi::DigitalOut mogo(MOGO_PORT);
inline pros::adi::DigitalOut doinker(DOINKER_PORT);

inline pros::Imu imu(IMU_PORT);
inline pros::Optical optical(OPTICAL_PORT);

inline chisel::DriveTrain drive_train(
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    15, // track width
    450 // wheel rpm
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

inline chisel::PIDSettings angular_pid_settings{
    10, 0, 0, 0, 0, 999, 0, 0, 0
};
inline std::atomic<float> target_heading (0);
inline std::atomic<float> angular_pid_output (0);
inline chisel::PIDController angular_pid_controller {
    odom.pose.h,
    target_heading,
    angular_pid_output,
    angular_pid_settings,
    127,
    0,
    1000 * 60 * 20,
    chisel::deg_err
};

inline chisel::PIDSettings lateral_pid_settings{
    10, 0, 0, 0, 0, 999, 0, 0, 0
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
    127,
    0,
    1000 * 60 * 20,
    nullptr
};

inline chisel::Chassis chassis = {
    &drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_controller, &lateral_pid_controller,
    true
};

inline void device_update() {
    intake_itf.clean_commands();
    intake_itf.update();
    intake_itf.push_update();

    arm_pos.store(arm.get_position());
    chisel::pid_handle_process(arm_pid);
    (void)arm.move(arm_pid_output.load());
}