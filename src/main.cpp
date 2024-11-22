#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstddef>
#include <queue>
#include <atomic>
#include <string>
#include <sys/_stdint.h>

#define DIGI_BUTTON controller_digital_e_t
#define ANAL_BUTTON controller_analog_e_t

#define CTRL_ANAL_LX E_CONTROLLER_ANALOG_LEFT_X
#define CTRL_ANAL_LY E_CONTROLLER_ANALOG_LEFT_Y

#define CTRL_ANAL_RX E_CONTROLLER_ANALOG_RIGHT_X
#define CTRL_ANAL_RY E_CONTROLLER_ANALOG_RIGHT_Y

#define CTRL_DIGI_L1 E_CONTROLLER_DIGITAL_L1
#define CTRL_DIGI_L2 E_CONTROLLER_DIGITAL_L2
#define CTRL_DIGI_R1 E_CONTROLLER_DIGITAL_R1
#define CTRL_DIGI_R2 E_CONTROLLER_DIGITAL_R2

#define CTRL_DIGI_A E_CONTROLLER_DIGITAL_A
#define CTRL_DIGI_B E_CONTROLLER_DIGITAL_B
#define CTRL_DIGI_X E_CONTROLLER_DIGITAL_X
#define CTRL_DIGI_Y E_CONTROLLER_DIGITAL_Y

#define CTRL_DIGI_UP E_CONTROLLER_DIGITAL_UP
#define CTRL_DIGI_DOWN E_CONTROLLER_DIGITAL_DOWN
#define CTRL_DIGI_LEFT E_CONTROLLER_DIGITAL_LEFT
#define CTRL_DIGI_RIGHT E_CONTROLLER_DIGITAL_RIGHT

constexpr long PROCESS_DELAY = 15;
constexpr long LONG_DELAY = 34;

/**
 * Returns the sign of a value
 *
 * @param val value
 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * Stores the variables for a PID controller
 *
 * @param kP proportional component, provides the bulk of the power
 * @param kI integral component, adjusts for steady state error
 * @param kD derivative component, dampener
 * @param windup range around the target that the integral will start
 * accumulating, useful in large motions to prevent overshoot
 * @param slew maximum acceleration rate
 */
struct PIDController {
	float kP = 0;
	float kI = 0;
	float kD = 0;
	float windup = 0;
	float slew = 0;
};

/**
 * Calculates the power output of a PID given the current status
 *
 * @param error difference between current and target values
 * @param integral error accumulated over time
 * @param derivative dampener based on predicted future error
 * @param pid PID controller to calculate power with
 */
float calcPowerPID(
	int error, int integral, int derivative, const PIDController* pid
) {
	return error * pid->kP + integral * pid->kI + derivative * pid->kD;
}

/**
 * Standard PID process primarily for managing the integral and derivative
 * values
 *
 * @param value pointer to the current value
 * @param target pointer to the target value
 * @param timeout time allocated to task
 * @param pid PID controller to use
 * @param output pointer to location to store the output
 * @param normalize_func will be used to normalize the error given the value
 * and target, usually used when rotating in degrees
 */
void pid_process(
		std::atomic<float>* value,
		float target,
		const int32_t timeout,
		const PIDController* pid,
		std::atomic<float>* output,
		std::function<float(float, float)> normalize_func = nullptr
) {
	// start time of process
	int32_t start_time = pros::millis();
	
	// variables used for PID calculation
	float prev_output = 0;
	int prev_error = 0;
	int error = 0;
	int integral = 0;
	int derivative = 0;

	// while the process has not exceeded the time limit
	while(pros::millis() < start_time + timeout) {
		// update previous output
		prev_output = *output;

		// update error
		prev_error = error;
		error = target - *value;

		// check if we crossed the target or not by comparing the signs of
		// errors
		if (sgn(prev_error) != sgn(error)) {
			// if we did, reset integral
			integral = 0;
		}

		// check if the error is in the range of the integral windup
		if (std::abs(error) <= pid->windup) {
			// if so, update integral
			integral += error;
		} else {
			// if not, set integral to 0
			integral = 0;
		}

		// update derivative
		derivative = error - prev_error;

		// calculate the power
		float calc_power = calcPowerPID(error, integral, derivative, pid);
		// constrain the power to slew
		calc_power = std::max(prev_output - pid->slew,
				std::min(prev_error + pid->slew, calc_power));

		// set output power
		*output = calc_power;

		// delay to save resources
		pros::delay(PROCESS_DELAY);
	}
}

// structs

struct DriveCurve {
	float deadband = 0;
	float minOutput = 0;
	float expoCurve = 1;
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

PIDController arm_pid ({0.3, 0, 0});

// config controller

constexpr pros::DIGI_BUTTON POS_INFO_BUTTONS[] = {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_A};
constexpr pros::DIGI_BUTTON GENERAL_INFO_BUTTONS[] = {pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_B};

constexpr pros::ANAL_BUTTON DRIVE_JOYSTICK = pros::CTRL_ANAL_LY;
constexpr pros::ANAL_BUTTON TURN_JOYSTICK = pros::CTRL_ANAL_RX;

constexpr pros::DIGI_BUTTON INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L2;
constexpr pros::DIGI_BUTTON INTAKE_REV_BUTTON = pros::CTRL_DIGI_L1;

constexpr pros::DIGI_BUTTON MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::DIGI_BUTTON MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::DIGI_BUTTON BAR_ON_BUTTON = pros::CTRL_DIGI_X;
constexpr pros::DIGI_BUTTON BAR_OFF_BUTTON = pros::CTRL_DIGI_Y;

constexpr pros::DIGI_BUTTON ARM_UP_BUTTON = pros::CTRL_DIGI_UP;
constexpr pros::DIGI_BUTTON ARM_DOWN_BUTTON = pros::CTRL_DIGI_DOWN;

constexpr pros::DIGI_BUTTON ARM_END_ON_BUTTON = pros::CTRL_DIGI_RIGHT;
constexpr pros::DIGI_BUTTON ARM_END_OFF_BUTTON = pros::CTRL_DIGI_LEFT;

// config drive

DriveCurve drive_curve ({3, 10, 2});
DriveCurve turn_curve ({3, 10, 2});

int drive_ratio = 1;
int turn_ratio = 1;

bool speed_comp = true;

// config auton

PIDController lateral_pid ({1, 0, 0});

PIDController angular_pid ({0.5, 0.02, 5});

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
std::atomic<float> heading (0);

long last_pos_update = 0;

void get_robot_position(long last_update) {
	last_pos_update = last_update;

	xPos.fetch_add(imu.get_accel().x);
	yPos.fetch_add(imu.get_accel().y);

	heading.store(imu.get_heading());
}

// telemetry
constexpr size_t TELEMTRY_SIZE = 100;
std::string telemetry[TELEMTRY_SIZE];

bool check_multi_DIGI_BUTTON(int n, const pros::DIGI_BUTTON* DIGI_BUTTONs) {
	for (int i = 0; i < n; ++i) if (!master.get_digital(DIGI_BUTTONs[n])) return false;

	return true;
}

void update_telemetry() {
	master.clear_line(0);
	master.clear_line(1);
	master.clear_line(2);

	if (check_multi_DIGI_BUTTON(3, GENERAL_INFO_BUTTONS)) {
		master.set_text(0, 0, "VEX2024-938E");
		master.set_text(1, 0, "time " + std::to_string(pros::millis()));
		master.set_text(2, 0, "batt " + std::to_string(master.get_battery_capacity()));
	} else if (check_multi_DIGI_BUTTON(3, POS_INFO_BUTTONS)) {
		master.set_text(0, 0, "xPos " + std::to_string(xPos.load()));
		master.set_text(1, 0, "yPos " + std::to_string(yPos.load()));
		master.set_text(2, 0, "head " + std::to_string(heading.load()));
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
            pros::lcd::print(2, "head: %f", heading.load());

            pros::delay(PROCESS_DELAY);
        }
    });
}

void disabled() {}

void competition_initialize() {}

struct Pose {
	float xPos = 0;
	float yPos = 0;
	float head = 0;
	bool forward = true;
};

int mod(int a, int b) {
    return (a % b + b) % b;
}

/*
**
 * Rotates the robot to a heading
 * 
 * @param target_heading heading to turn to
 * @param timeout time allocated to the process
 *
void turn_to_heading(float target_heading, int32_t timeout) {
	// variables used for PID calculation
	int prev_error = 0;
	int error = 0;
	int integral = 0;
	int derivative = 0;

	// direction to turn in; true for clockwise and false for counterclockwise
	bool turn_dir;

	// calculates the distance for turning clockwise and counterclockwise
	float clockwise_diff = mod(target_heading - heading.load(), 360);
	float counterclockwise_diff = mod(heading.load() - target_heading, 360);

	// compares the errors and selects the best option, setting turn_dur and error
	if (clockwise_diff <= counterclockwise_diff) {
		turn_dir = true;
		error = clockwise_diff;
	} else {
		turn_dir = false;
		error = -counterclockwise_diff;
	}

	// when the process should timeout
	int32_t end_time = pros::millis() + timeout;

	// turning toward target heading
	while(true) {
		// updating values
		prev_error = error;
		if (turn_dir) error = mod(target_heading - heading.load(), 360);
		else error = -mod(heading.load() - target_heading, 360);
		integral += error;
		derivative = error - prev_error;

		// check if we crossed the target heading
		if (turn_dir && mod(heading.load() - target_heading, 360) < mod(target_heading - heading.load(), 360)
		|| !turn_dir && mod(target_heading - heading.load(), 360) < mod(heading.load() - target_heading, 360)) {
			// if so we flip the turn direction and set integral to 0;
			turn_dir = !turn_dir;
			integral = 0;
		}

		// calculate the power that should be put into the motors
		float calc_power = calcPowerPID(error, integral, derivative, angular_pid);

		// run the motors at the calculated power
		dt_left_motors.move(calc_power);
		dt_right_motors.move(-calc_power);

		// check if we should end the process
		if (pros::millis() >= end_time) break;

		pros::delay(PROCESS_DELAY);
	}
}
*/

void turn_to_heading(float target_heading, int32_t timeout) {
	// error normalization function to deal with 0-360 wrap around
	auto normalize_rotation = [](float target, float current) {
		float error = target - current;
		return fmod((error + 180), 360) - 180;
	};

	// records the output of the PID process
	std::atomic<float> output (0);

	// start time of process
	int32_t start_time = pros::millis();

	// start the PID process
	pros::Task pid_process_task{[&] {
		pid_process(
				&heading,
				target_heading,
				timeout,
				&angular_pid,
				&output,
				normalize_rotation
		);
	}};
	
	// apply the motor values
	while (pros::millis() < start_time + timeout) {
		dt_left_motors.move(output.load());
		dt_right_motors.move(-output.load());

		// delay to save resources
		pros::delay(PROCESS_DELAY);
	}

	// brake motors at end of process
	dt_left_motors.brake();
	dt_right_motors.brake();
}

void autonomous() {
	log("auton start");

	turn_to_heading(90, 3000);

	turn_to_heading(0, 3000);

	dt_left_motors.brake();
	dt_right_motors.brake();
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

std::atomic<int> heading_arm_pos = 0;

void op_async() {
	int prev_error = 0;
	int error = 0;
	int integral = 0;
	int derivative = 0;

	while(true) {
		if (heading_arm_pos.load() < 0) heading_arm_pos.store(0);

		prev_error = error;
		error = heading_arm_pos.load() - arm.get_position();
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
		if (master.get_digital(ARM_UP_BUTTON)) heading_arm_pos += ARM_SPEED;
		else if (master.get_digital(ARM_DOWN_BUTTON)) heading_arm_pos -= ARM_SPEED;

		// arm end
		if(master.get_digital(ARM_END_ON_BUTTON)) arm_end.set_value(true);
		else if(master.get_digital(ARM_END_OFF_BUTTON)) arm_end.set_value(false);

        pros::delay(PROCESS_DELAY);
    }
}