#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <cmath>
#include <cstddef>

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

// config motors

constexpr int INTAKE_MOTOR_SPEED = 127;

constexpr int ARM_SPEED = 127;
constexpr int ARM_IDLE_SPEED = 30;

// config controller

constexpr pros::controller_analog_e_t DRIVE_JOYSTICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::controller_analog_e_t TURN_JOYSTICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::controller_digital_e_t INTAKE_FWD_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;
constexpr pros::controller_digital_e_t INTAKE_REV_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;

constexpr pros::controller_digital_e_t MOGO_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
constexpr pros::controller_digital_e_t MOGO_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_B;

constexpr pros::controller_digital_e_t BAR_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
constexpr pros::controller_digital_e_t BAR_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_Y;

constexpr pros::controller_digital_e_t ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
constexpr pros::controller_digital_e_t ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_DOWN;

constexpr pros::controller_digital_e_t ARM_END_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr pros::controller_digital_e_t ARM_END_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_LEFT;

// config drive

struct DriveCurve {
	float deadband = 0;
	float minOutput = 0;
	float expoCurve = 1;
};

DriveCurve drive_curve ({3, 10, 2});
DriveCurve turn_curve ({3, 10, 2});

int drive_ratio = 1;
int turn_ratio = 1;

bool speed_comp = true;

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

pros::adi::DigitalOut mogo_piston (MOGO_PORT);
pros::adi::DigitalOut bar_piston (BAR_PORT);

pros::Motor arm (ARM_PORT, pros::v5::MotorGears::red);
pros::adi::DigitalOut arm_end (ARM_END_PORT);

pros::IMU imu (IMU_PORT);

// robot position
float xPos = 0;
float yPos = 0;
float head = 0;

void initialize() {
	pros::lcd::initialize();
    
	master.set_text(0, 0, "calibrating imu");
	imu.reset(true);
	master.set_text(0, 0, "imu calibrated");

    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "xPos: %f", xPos);
            pros::lcd::print(1, "yPos: %f", yPos);
            pros::lcd::print(2, "head: %f", head);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

float calcPower(int value, int other_value, DriveCurve curve, int ratio, int other_ratio) {
	if (std::abs(value) <= curve.deadband) return 0;

	float ratio_mult = (float) ratio / (ratio + other_ratio);
	if (speed_comp) {
		float dynamic_ratio = ((float) other_ratio * std::abs(other_value) / 127);
		ratio_mult = (float) ratio / (ratio + dynamic_ratio);
	}

	int sign_mult = std::copysign(1, value);

	float adj = 127 / std::pow(127, curve.expoCurve);
	float expo = std::pow(std::abs(value), curve.expoCurve);

	float output = (float) (adj * expo);
	
	return sign_mult * std::max(curve.minOutput, output);
}

void opcontrol() {
	master.set_text(0, 0, "op control");

    while (true) {
		// driving
        int drive_value = master.get_analog(DRIVE_JOYSTICK);
        int turn_value = master.get_analog(TURN_JOYSTICK);

		float drive_power = calcPower(drive_value, turn_value, drive_curve, drive_ratio, turn_ratio);
		float turn_power = calcPower(turn_value, drive_value, turn_curve, turn_ratio, drive_ratio);

		dt_left_motors.move(drive_power + turn_power);
		dt_right_motors.move(drive_power - turn_power);

		// intake
		if (master.get_digital(INTAKE_FWD_BUTTON)) intake_motor.move(INTAKE_MOTOR_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake_motor.move(-INTAKE_MOTOR_SPEED);
		else intake_motor.brake();
		
		// mogo
		if (master.get_digital(MOGO_ON_BUTTON)) mogo_piston.set_value(true);
		else if (master.get_digital(MOGO_OFF_BUTTON)) mogo_piston.set_value(false);
		
		// bar
		if (master.get_digital(BAR_ON_BUTTON)) bar_piston.set_value(true);
		else if (master.get_digital(BAR_OFF_BUTTON)) bar_piston.set_value(false);

		// arm
		if (master.get_digital(ARM_UP_BUTTON)) arm.move(ARM_SPEED);
		else if (master.get_digital(ARM_DOWN_BUTTON)) arm.move(-ARM_SPEED);
		else arm.move(ARM_IDLE_SPEED);

		// arm end

		if(master.get_digital(ARM_END_ON_BUTTON)) arm_end.set_value(true);
		else if(master.get_digital(ARM_END_OFF_BUTTON)) arm_end.set_value(false);

        pros::delay(25);
    }
}