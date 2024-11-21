#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <cstddef>
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

// config ports

constexpr int DT_FL_PORT = -15;
constexpr int DT_LM_PORT = -1;
constexpr int DT_BL_PORT = -20;

constexpr int DT_FR_PORT = 14;
constexpr int DT_MR_PORT = 18;
constexpr int DT_BR_PORT = 16;

constexpr int INTAKE_PORT = -2;

constexpr int MOGO_PORT = 1;
constexpr int BAR_PORT = 3;

constexpr int ARM_PORT = -5;
constexpr int ARM_END_PORT = 8;

constexpr int IMU_PORT = 21;

// config

constexpr int intake_motor_SPEED = 127;
constexpr int ARM_SPEED = 127;

// defs

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup dt_left_motors ({
	DT_FL_PORT,
	DT_LM_PORT,
	DT_BL_PORT
}, pros::MotorGearset::blue);

pros::MotorGroup dt_right_motors ({
	DT_FR_PORT,
	DT_MR_PORT,
	DT_BR_PORT
}, pros::MotorGearset::blue);

pros::Motor intake_motor (INTAKE_PORT, pros::v5::MotorGears::green);

pros::adi::DigitalOut mogo_clamp_piston (MOGO_PORT);
pros::adi::DigitalOut mogo_bar_piston (BAR_PORT);

pros::Motor arm (ARM_PORT, pros::v5::MotorGears::red);
pros::adi::DigitalOut arm_end (ARM_END_PORT);

pros::IMU imu (IMU_PORT);

void initialize() {
	pros::lcd::initialize();
    

	imu.reset();

    pros::Task screen_task([&]() {
        while (true) {
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {	

}
void autonomous() {
}

void opcontrol() {

    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake_motor.move(intake_motor_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake_motor.move(-intake_motor_SPEED);
		else intake_motor.brake();
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) mogo_clamp_piston.set_value(true);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) mogo_clamp_piston.set_value(false);
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) arm.move(ARM_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) arm.move(-ARM_SPEED);
		else arm.brake();

		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
		// 	arm_end.set_value(true);
		// 	arm.set_value(true);
		// }
		// else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		// 	arm.set_value(false);
		// }
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
		arm_end.set_value(true);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
		arm_end.set_value(false);
		}

        pros::delay(25);
    }
}