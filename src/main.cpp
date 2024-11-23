#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <queue>
#include <atomic>
#include <string>
#include <sys/_stdint.h>

#define digi_button controller_digital_e_t
#define anal_button controller_analog_e_t

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

#define Cartridge MotorGearset

#define DEL80 "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b"

// If these are change they will probably screw up the entire PID system, so
// unless you desperately need to, don't.
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

void printc_bulk(char c, int n) {
	for (int i = 0; i < n; ++i) printf("%c", c);
}



/*****************************************************************************/
/*                                  CONFIG                                   */
/*****************************************************************************/

// PORTS

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

// BUTTONS

constexpr pros::anal_button DRIVE_JOYSTICK = pros::CTRL_ANAL_LY;
constexpr pros::anal_button TURN_JOYSTICK = pros::CTRL_ANAL_RX;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L2;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::CTRL_DIGI_L1;

constexpr pros::digi_button MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::digi_button MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::digi_button BAR_ON_BUTTON = pros::CTRL_DIGI_X;
constexpr pros::digi_button BAR_OFF_BUTTON = pros::CTRL_DIGI_Y;

constexpr pros::digi_button ARM_UP_BUTTON = pros::CTRL_DIGI_UP;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::CTRL_DIGI_DOWN;

constexpr pros::digi_button ARM_END_ON_BUTTON = pros::CTRL_DIGI_RIGHT;
constexpr pros::digi_button ARM_END_OFF_BUTTON = pros::CTRL_DIGI_LEFT;

// PID CONTROLLER

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

PIDController lateral_pid ({1, 0, 0});
PIDController angular_pid ({0.5, 0.02, 5});

PIDController arm_pid ({0.3, 0, 0, 0, 999});

// DRIVING

struct DriveCurve {
	float deadband = 0;
	float minOutput = 0;
	float expoCurve = 1;
};

DriveCurve drive_curve ({3, 10, 2});
DriveCurve turn_curve ({3, 10, 2});

constexpr int drive_ratio = 1;
constexpr int turn_ratio = 1;

constexpr bool speed_comp = true;

constexpr int INTAKE_MOTOR_SPEED = 127;

constexpr int ARM_SPEED = 50;
constexpr float ARM_DOWN_SPEED_MULTI = 0.5;
constexpr float MIN_ARM_HEIGHT = 0;
constexpr float MAX_ARM_HEIGHT = 2500;

float calcPowerCurve(
		int value,
		int other_value,
		DriveCurve curve,
		int ratio,
		int other_ratio
) {
	if (std::abs(value) <= curve.deadband) return 0;

	float ratio_mult = (float) ratio / (ratio + other_ratio);
	if (speed_comp) {
		float dynamic_ratio = ((float) other_ratio * std::abs(other_value) / 127);
		ratio_mult = (float) ratio / (ratio + dynamic_ratio);
	}

	int sign_mult = sgn(value);

	float adj = 127 / std::pow(127, curve.expoCurve);
	float expo = std::pow(std::abs(value), curve.expoCurve);

	float output = (float) (adj * expo);
	
	return sign_mult * std::max(curve.minOutput, output);
}



/*****************************************************************************/
/*                             MICROCONTROLLERS                              */
/*****************************************************************************/

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup dt_left_motors ({
	DT_FL_PORT,
	DT_LM_PORT,
	DT_BL_PORT
}, pros::Cartridge::blue);

pros::MotorGroup dt_right_motors ({
	DT_FR_PORT,
	DT_MR_PORT,
	DT_BR_PORT
}, pros::Cartridge::blue);

pros::Motor intake_motor (INTAKE_PORT, pros::Cartridge::green);

pros::adi::DigitalOut mogo_piston (MOGO_PORT);
pros::adi::DigitalOut bar_piston (BAR_PORT);

pros::Motor arm (ARM_PORT, pros::Cartridge::red);
pros::adi::DigitalOut arm_end (ARM_END_PORT);

pros::IMU imu (IMU_PORT);



/*****************************************************************************/
/*                               PID CONTROLLER                              */
/*****************************************************************************/

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

int pid_process_counter = 0;

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
		std::atomic<float>* target,
		const int32_t timeout,
		const PIDController* pid,
		std::atomic<float>* output,
		std::function<float(float, float)> normalize_func = nullptr
) {
	int process_number = pid_process_counter++;
	printf("[%d]: new PID process %d started\n", pros::millis(), process_number);

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
		printf("[%d]: PID process %d running\n", pros::millis(), process_number);
		printf("[%d]: PID process %d running\n", pros::millis(), process_number);

		// update previous output
		prev_output = *output;

		// update error
		prev_error = error;
		error = *target - *value;

		// apply normalization if provided
        if (normalize_func) error = normalize_func(*target, *value);

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
		printf("[%d]: PID process %d output set to %f\n", pros::millis(), process_number, output->load());

		// delay to save resources
		pros::delay(PROCESS_DELAY);
	}
}



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



/*****************************************************************************/
/*                                COMPETITION                                */
/*****************************************************************************/

void initialize() {
	pros::lcd::initialize();
    
	auto start = pros::millis();

	imu.reset();
	while(imu.is_calibrating()) {
		pros::delay(LONG_DELAY);
		printf("resetting imu: %dms elapsed\n", pros::millis() - start);
	}
	printf("imu reset completed, took %dms\n", pros::millis() - start);

    pros::Task pos_tracking_task([&]() {
		last_pos_update = pros::millis();

        while (true) {
			get_robot_position(last_pos_update);
			
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
		// pid_process(
		// 		&heading,
		// 		&target_heading,
		// 		timeout,
		// 		&angular_pid,
		// 		&output,
		// 		normalize_rotation
		// );
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

	// remove task
	pid_process_task.remove();
}

void autonomous() {
	turn_to_heading(90, 3000);

	turn_to_heading(0, 3000);

	dt_left_motors.brake();
	dt_right_motors.brake();
}

std::atomic<float> arm_target_pos = 0;

void opcontrol() {
	printf("op control start\n");

	// dampens the error when moving downward to prevent dropping the arm
	auto error_mod = [](float target, float current) {
		float error = target - current;
		if (error < 0) error *= ARM_DOWN_SPEED_MULTI;
		return error;
	};
	// records the output of the arm PID process
	std::atomic<float> output (0);
	// records position of the arm throughout the PID process
	std::atomic<float> arm_pos (0);

	// start arm PID process
	pros::Task arm_pid_task([&] {
		pid_process(
				&arm_pos,
				&arm_target_pos,
				120000, // bombastically long timeout to cover entire period
				&arm_pid,
				&output,
				error_mod
		);
	});

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
		if (master.get_digital(ARM_UP_BUTTON)) arm_target_pos += ARM_SPEED;
		else if (master.get_digital(ARM_DOWN_BUTTON)) arm_target_pos -= ARM_SPEED;
		// constrain the target arm pos
		arm_target_pos.store(std::min(MAX_ARM_HEIGHT, std::max(MIN_ARM_HEIGHT, arm_target_pos.load())));
		// update current arm pos
		arm_pos.store(arm.get_position());
		// move arm to PID output
		arm.move(output.load());

		// arm end
		if(master.get_digital(ARM_END_ON_BUTTON)) arm_end.set_value(true);
		else if(master.get_digital(ARM_END_OFF_BUTTON)) arm_end.set_value(false);

        pros::delay(PROCESS_DELAY);
    }
}