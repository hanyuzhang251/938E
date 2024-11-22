#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>
#include <queue>
#include <atomic>
#include <string>

#define digi_button controller_digital_e_t
#define anal_button controller_analog_e_t

constexpr double INCH_PER_G = 386.08858267717;

constexpr long PROCESS_DELAY = 15;
constexpr long LONG_DELAY = 34;

// structs

struct DriveCurve {
	float deadband = 0;
	float minOutput = 0;
	float expoCurve = 1;
};

struct PIDController {
	float kP = 0;
	float kI = 0;
	float kD = 0;
	float small_error = 1;
};

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

constexpr int ARM_SPEED = 50;

PIDController arm_pid ({0.3, 0, 0, 30});

// config controller

constexpr pros::digi_button POS_INFO_BUTTONS[] = {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_A};
constexpr pros::digi_button GENERAL_INFO_BUTTONS[] = {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_B};

constexpr pros::anal_button DRIVE_JOYSTICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::anal_button TURN_JOYSTICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;

constexpr pros::digi_button MOGO_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
constexpr pros::digi_button MOGO_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_B;

constexpr pros::digi_button BAR_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
constexpr pros::digi_button BAR_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_Y;

constexpr pros::digi_button ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_DOWN;

constexpr pros::digi_button ARM_END_ON_BUTTON = pros::E_CONTROLLER_DIGITAL_RIGHT;
constexpr pros::digi_button ARM_END_OFF_BUTTON = pros::E_CONTROLLER_DIGITAL_LEFT;

// config drive

DriveCurve drive_curve ({3, 10, 2});
DriveCurve turn_curve ({3, 10, 2});

int drive_ratio = 1;
int turn_ratio = 1;

bool speed_comp = true;

// config auton

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

	xPos.fetch_add(imu.get_accel().x);
	yPos.fetch_add(imu.get_accel().y);

	head.store(imu.get_heading());
}

// telemetry
constexpr size_t TELEMTRY_SIZE = 100;
std::string telemetry[TELEMTRY_SIZE];

bool check_multi_digi_button(int n, const pros::digi_button* digi_buttons) {
	for (int i = 0; i < n; ++i) if (!master.get_digital(digi_buttons[n])) return false;

	return true;
}

void update_telemetry() {
	master.clear_line(0);
	master.clear_line(1);
	master.clear_line(2);

	if (check_multi_digi_button(3, GENERAL_INFO_BUTTONS)) {
		master.set_text(0, 0, "VEX2024-938E");
		master.set_text(1, 0, "time " + std::to_string(pros::millis()));
		master.set_text(2, 0, "batt " + std::to_string(master.get_battery_capacity()));
	} else if (check_multi_digi_button(3, POS_INFO_BUTTONS)) {
		master.set_text(0, 0, "xPos " + std::to_string(xPos.load()));
		master.set_text(1, 0, "yPos " + std::to_string(yPos.load()));
		master.set_text(2, 0, "head " + std::to_string(head.load()));
	} else {
		master.set_text(0, 0, telemetry[0]);
		master.set_text(1, 0, telemetry[1]);
		master.set_text(2, 0, telemetry[2]);
	}
}

void log(std::string str) {
	for (int i = TELEMTRY_SIZE - 1; i > 0; --i) {
		telemetry[i] = telemetry[i - 1];
	}

	telemetry[0] = str;
}

void amend_last_log(std::string str) {
	telemetry[0] = str;

	master.set_text(0, 0, telemetry[0]);
}

// competition:

void initialize() {
	log("initialize");

	pros::lcd::initialize();
    
	log("reset imu");
	auto start = pros::millis();

	imu.reset();
	while(imu.is_calibrating()) {
		amend_last_log("reset imu " + std::to_string(pros::millis()));
		pros::delay(PROCESS_DELAY);
	}

	amend_last_log("imu reset " +  std::to_string(pros::millis()));

    pros::Task pos_tracking_task([&]() {
		last_pos_update = pros::millis();

        while (true) {
			get_robot_position(last_pos_update);

			if (master.get_digital(MOGO_ON_BUTTON))
			update_telemetry();
			
            pros::lcd::print(0, "xPos: %f", xPos.load());
            pros::lcd::print(1, "yPos: %f", yPos.load());
            pros::lcd::print(2, "head: %f", head.load());

            pros::delay(LONG_DELAY);
        }
    });
}

void disabled() {}

void competition_initialize() {}

float calcPowerPID(int error, int integral, int derivative, PIDController pid_controller) {
	float power;

	power = error * pid_controller.kP + integral * pid_controller.kI + derivative * pid_controller.kP;

	return power;
}

struct Pose {
	float xPos = 0;
	float yPos = 0;
	float head = 0;
	bool forward = true;
};

std::queue<Pose> instructions;

void adjust_angle(Pose target) {
	if (!target.forward) {
		target.head += 180;
		if (target.head >= 360) target.head -= 360;
	}

	log("adj. angle from " + std::to_string(head) + " to " + std::to_string(target.head));

	int prev_error = 0;
	int error = 0;
	int integral = 0;
	int derivative = 0;

	while(true) {
		prev_error = error;

		float right_side_error = std::abs(target.head - head.load());
		float left_side_error = std::abs(std::min(target.head, head.load()) + 360 - std::max(target.head, head.load()));

		if (right_side_error < left_side_error) error = right_side_error;
		else error = -left_side_error;

		integral += error;
		derivative = error - prev_error;

		if (std::abs(error) >= angular_pid.small_error) {
			float calc_power = calcPowerPID(error, integral, derivative, angular_pid);

			dt_left_motors.move(calc_power);
			dt_right_motors.move(-calc_power);
		} else {;
			dt_left_motors.brake();
			dt_right_motors.brake();
			break;
		}

		pros::delay(PROCESS_DELAY);
	}
}

void auton_async() {
	log("async auton start");

	Pose target;

	while (true) {
		pros::delay(PROCESS_DELAY);

		if (instructions.empty()) continue;

		Pose target = instructions.front();

		adjust_angle(target);
	}
}

void autonomous() {
	log("auton start");

	pros::Task auton_task(auton_async);

	log("end async auton");
	auton_task.remove();
	amend_last_log("async auton end");
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

std::atomic<int> target_arm_pos = 0;

void op_async() {
	int prev_error = 0;
	int error = 0;
	int integral = 0;
	int derivative = 0;

	while(true) {
		if (target_arm_pos.load() < 0) target_arm_pos.store(0);

		prev_error = error;
		error = target_arm_pos.load() - arm.get_position();
		integral += error;
		derivative = error - prev_error;

		if (std::abs(error) <= arm_pid.small_error) {
			integral = 0;
		}

		float calc_power = calcPowerPID(error, integral, derivative, arm_pid);

		if (calc_power < 0) calc_power /= 2;

		arm.move(calc_power);

		pros::delay(PROCESS_DELAY);
	}
}

void opcontrol() {
	log("op control started");

	pros::Task op_async_task(op_async);

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
		else if (master.get_digital(INTAKE_REV_BUTTON)) intake_motor.move(-INTAKE_MOTOR_SPEED);
		else intake_motor.brake();
		
		// mogo
		if (master.get_digital(MOGO_ON_BUTTON)) mogo_piston.set_value(true);
		else if (master.get_digital(MOGO_OFF_BUTTON)) mogo_piston.set_value(false);
		
		// bar
		if (master.get_digital(BAR_ON_BUTTON)) bar_piston.set_value(true);
		else if (master.get_digital(BAR_OFF_BUTTON)) bar_piston.set_value(false);

		// arm
		if (master.get_digital(ARM_UP_BUTTON)) target_arm_pos += ARM_SPEED;
		else if (master.get_digital(ARM_DOWN_BUTTON)) target_arm_pos -= ARM_SPEED;

		// arm end
		if(master.get_digital(ARM_END_ON_BUTTON)) arm_end.set_value(true);
		else if(master.get_digital(ARM_END_OFF_BUTTON)) arm_end.set_value(false);

        pros::delay(PROCESS_DELAY);
    }
}