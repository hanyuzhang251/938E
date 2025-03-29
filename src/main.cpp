#include "main.hpp"
#include "devices.h"
#include "skills.h"

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
    run_auton();
}

void opcontrol() {
    while (true) {
        const int32_t lateral_move = chassis.lateral_drive_settings->drive_calc_power(master.get_analog(ANALOG_LEFT_Y));
        const int32_t angular_move = chassis.angular_drive_settings->drive_calc_power(master.get_analog(ANALOG_LEFT_X));

        (void)left_motors.move(lateral_move + angular_move);
        (void)left_motors.move(lateral_move - angular_move);
    }
}