#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>
#include <queue>
#include <atomic>

constexpr double INCH_PER_G = 386.08858267717;

constexpr long PROCESS_DELAY = 15;

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

// config auton

struct PIDController {
	float kP = 0;
	float kI = 0;
	float kD = 0;
	float small_error = 1;
};

PIDController lateral_pid ({0, 0, 0, 1});
PIDController angular_pid ({0, 0, 0, 3});

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
std::atomic<float> xPos (0);
std::atomic<float> yPos (0);
std::atomic<float> head (0);

long last_pos_update = 0;

void get_robot_position(long last_update) {
	last_pos_update = last_update;

	xPos.fetch_add(imu.get_accel().x * INCH_PER_G);
	yPos.fetch_add(imu.get_accel().y *INCH_PER_G);

	head.store(imu.get_heading());
}

// telemetry
constexpr size_t TELEMTRY_SIZE = 100;
std::string telemetry[TELEMTRY_SIZE];

void log(std::string str) {
	for (int i = TELEMTRY_SIZE - 1; i > 0; --i) {
		telemetry[i] = telemetry[i - 1];
	}

	telemetry[0] = str;

	master.set_text(0, 0, telemetry[0]);
	master.set_text(1, 0, telemetry[1]);
	master.set_text(2, 0, telemetry[2]);
}

void amend(std::string str) {
	telemetry[0] = str;

	master.set_text(0, 0, telemetry[0]);
}

// competition:

void initialize() {
	log("initializing");

	pros::lcd::initialize();
    
	log("calibrating imu");
	auto start = pros::millis();

	imu.reset();
	while(imu.is_calibrating()) {
		amend("calibrating imu " + std::to_string(pros::millis()));
		pros::delay(PROCESS_DELAY);
	}

	amend("imu calibrated " +  std::to_string(pros::millis()));

    pros::Task pos_tracking_task([&]() {
		last_pos_update = pros::millis();

        while (true) {
			get_robot_position(last_pos_update);
			
            pros::lcd::print(0, "xPos: %f", xPos.load());
            pros::lcd::print(1, "yPos: %f", yPos.load());
            pros::lcd::print(2, "head: %f", head.load());

            pros::delay(PROCESS_DELAY);
        }
    });
}

void disabled() {}

void competition_initialize() {}

float calcPowerPID(int error, int integral, PIDController pid_controller) {
	float power;

	power = error * pid_controller.kP + integral * pid_controller.kI;
}

struct Pose {
	float xPos = 0;
	float yPos = 0;
	float head = 0;
};

std::queue<Pose> instructions;

void auton_async() {
	log("async auton started");

	Pose target;

	int error = 0;
	int integral = 0;

	while (true) {
		pros::delay(PROCESS_DELAY);

		if (instructions.empty()) continue;

		Pose target = instructions.front();

	
		float right_side_error = std::abs(target.head - head.load());
		float left_side_error = std::abs(std::min(target.head, head.load()) + 360 - std::max(target.head, head.load()));
		if (error >= angular_pid.small_error) {
			dt_left_motors.move(calcPowerPID(error, integral, angular_pid));
		}
	}
}

void autonomous() {
	log("auton stated");

	pros::Task auton_task(auton_async);

	auton_task.remove();
}

float calcPowerCurve(int value, int other_value, DriveCurve curve, int ratio, int other_ratio) {
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
	log("op control started");

    while (true) {
		// driving
        int drive_value = master.get_analog(DRIVE_JOYSTICK);
        int turn_value = master.get_analog(TURN_JOYSTICK);

		float drive_power = calcPowerCurve(drive_value, turn_value, drive_curve, drive_ratio, turn_ratio);
		float turn_power = calcPowerCurve(turn_value, drive_value, turn_curve, turn_ratio, drive_ratio);

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