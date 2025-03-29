#include "main.hpp"
#include "ports.h"

#include "chisel/util/motoritf.h"

pros::MotorGroup left_motors ({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
pros::MotorGroup right_motors ({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

pros::Motor intake (INTAKE_PORT);
auto intake_itf = chisel::MotorItf(&intake);
pros::Motor arm (ARM_PORT);

pros::Imu imu (IMU_PORT);
pros::Optical optical (OPTICAL_PORT);

chisel::DriveTrain drive_train (
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    15, // track width
    450 // wheel rpm
);

chisel::DriveSettings lateral_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};
chisel::DriveSettings angular_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};

chisel::Odom odom {
    {0, 0, 0},
    {0, 0, 0},
    &imu,
    &drive_train,
    nullptr,
    0
};

chisel::PIDSettings angular_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::PIDSettings lateral_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::Chassis chassis = {&drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_settings, &lateral_pid_settings, true};

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool init_done = false;

void init() {
    if (init_done) {
        return;
    }

    (void)left_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);
    (void)right_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);

    optical.set_integration_time(PROCESS_DELAY);

    pros::lcd::initialize();
    master.clear();

    chassis.initialize();
}

void initialize() {
    init();
}

void competition_initialize() {
    init();
}

void autonomous() {
}

void opcontrol() {
    while (true) {
        const int32_t lateral_move = drive_calc_power(master.get_analog(ANALOG_LEFT_Y), *chassis.lateral_drive_settings);
        const int32_t angular_move = drive_calc_power(master.get_analog(ANALOG_LEFT_X), *chassis.angular_drive_settings);

        // (void)left_motors.move(lateral_move + angular_move);
        // (void)left_motors.move(lateral_move - angular_move);
    }
}