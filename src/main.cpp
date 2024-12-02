#include "main.h"
#include <atomic>

#define digi_button controller_digital_e_t
#define anlg_button controller_analog_e_t

#define CTRL_ANLG_LX E_CONTROLLER_ANALOG_LEFT_X
#define CTRL_ANLG_LY E_CONTROLLER_ANALOG_LEFT_Y

#define CTRL_ANLG_RX E_CONTROLLER_ANALOG_RIGHT_X
#define CTRL_ANLG_RY E_CONTROLLER_ANALOG_RIGHT_Y

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



/*****************************************************************************/
/*                                   UTIL                                    */
/*****************************************************************************/

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



/*****************************************************************************/
/*                                  CONFIG                                   */
/*****************************************************************************/

constexpr long PROCESS_DELAY = 15;
constexpr long LONG_DELAY = 200;

// PORTS

constexpr int DT_FL_PORT = -15;
constexpr int DT_LM_PORT = -1;
constexpr int DT_BL_PORT = -20;

constexpr int DT_FR_PORT = 14;
constexpr int DT_MR_PORT = 18;
constexpr int DT_BR_PORT = 16;

constexpr int INTAKE_PORT = -2;

constexpr int MOGO_PORT = 1;

constexpr int ARM_PORT = -5;
constexpr int ARM_END_PORT = 8;

constexpr int IMU_PORT = 21;

// BUTTONS

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::CTRL_ANLG_LY;
constexpr pros::anlg_button TURN_JOYSTICK = pros::CTRL_ANLG_RX;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::CTRL_DIGI_L2;

constexpr pros::digi_button MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::digi_button MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::digi_button ARM_UP_BUTTON = pros::CTRL_DIGI_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::CTRL_DIGI_R2;

constexpr pros::digi_button ARM_END_ON_BUTTON = pros::CTRL_DIGI_RIGHT;
constexpr pros::digi_button ARM_END_OFF_BUTTON = pros::CTRL_DIGI_LEFT;

constexpr pros::digi_button RESET_ARM_POS_BUTTON = pros::CTRL_DIGI_X;
constexpr pros::digi_button FORCE_ARM_POS_BUTTON = pros::CTRL_DIGI_Y;

// PID CONTROLLERS

constexpr float LATERAL_PID_KP = 0.07;
constexpr float LATERAL_PID_KI = 0.005;
constexpr float LATERAL_PID_KD = 0;
constexpr float LATERAL_PID_WIND = 300;
constexpr float LATERAL_PID_SLEW = 999;

constexpr float ANGULAR_PID_KP = 0.8;
constexpr float ANGULAR_PID_KI = 0.1;
constexpr float ANGULAR_PID_KD = 3;
constexpr float ANGULAR_PID_WIND = 30;
constexpr float ANGULAR_PID_SLEW = 999;

constexpr float ARM_PID_KP = 0.3;
constexpr float ARM_PID_KI = 0.02;
constexpr float ARM_PID_KD = 0;
constexpr float ARM_PID_WIND = 90;
constexpr float ARM_PID_SLEW = 999;

// DRIVING

constexpr int DRIVE_RATIO = 1;
constexpr int TURN_RATIO = 1;

constexpr bool SPEED_COMP = false;

constexpr int intake_SPEED = 127;

constexpr float ARM_SPEED = 50;
constexpr float ARM_DOWN_SPEED_MULTI = 0.5;
constexpr float MIN_ARM_HEIGHT = 0;
constexpr float MAX_ARM_HEIGHT = 2700;

constexpr float DRIVE_CURVE_DEADBAND = 3;
constexpr float DRIVE_CURVE_MIN_OUT = 10;
constexpr float DRIVE_CURVE_EXPO_CURVE = 3;

constexpr float TURN_CURVE_DEADBAND = 3;
constexpr float TURN_CURVE_MIN_OUT = 10;
constexpr float TURN_CURVE_EXPO_CURVE = 3;



/*****************************************************************************/
/*                              PID CONTROLLER                               */
/*****************************************************************************/

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

PIDController lateral_pid ({
		LATERAL_PID_KP,
		LATERAL_PID_KI,
		LATERAL_PID_KD,
		LATERAL_PID_WIND,
		LATERAL_PID_SLEW
});

PIDController angular_pid ({
		ANGULAR_PID_KP,
		ANGULAR_PID_KI,
		ANGULAR_PID_KD,
		ANGULAR_PID_WIND,
		ANGULAR_PID_SLEW
});


PIDController arm_pid ({
		ARM_PID_KP,
		ARM_PID_KI,
		ARM_PID_KD,
		ARM_PID_WIND,
		ARM_PID_SLEW
});

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
		error = *target - *value;

		// apply normalization if provided
        if (normalize_func) error = normalize_func(*target, *value);

		// check if we crossed the target or not by comparing the signs of
		// errors
		if (sgn(prev_error) != sgn(error)) {
			// if we did, reset integral
			integral = 0;

			printf("reset integral\n");
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



/*****************************************************************************/
/*                                  DRIVING                                  */
/*****************************************************************************/

struct DriveCurve {
	float deadband = 0;
	float minOutput = 0;
	float expoCurve = 1;
};

DriveCurve drive_curve ({
		DRIVE_CURVE_DEADBAND,
		DRIVE_CURVE_MIN_OUT,
		DRIVE_CURVE_EXPO_CURVE
});

DriveCurve turn_curve ({
		TURN_CURVE_DEADBAND,
		TURN_CURVE_MIN_OUT,
		TURN_CURVE_EXPO_CURVE
});

float calcPowerCurve(
		int value,
		int other_value,
		DriveCurve curve,
		int ratio,
		int other_ratio
) {
	if (std::abs(value) <= curve.deadband) return 0;

	float ratio_mult = (float) ratio / (ratio + other_ratio);
	if (SPEED_COMP) {
		float dynamic_ratio = (
				(float) other_ratio * std::abs(other_value) / 127);
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
});

pros::MotorGroup dt_right_motors ({
	DT_FR_PORT,
	DT_MR_PORT,
	DT_BR_PORT
});

pros::Motor intake (INTAKE_PORT);

pros::adi::DigitalOut mogo (MOGO_PORT);

pros::Motor arm (ARM_PORT);
pros::adi::DigitalOut arm_end (ARM_END_PORT);

pros::IMU imu (IMU_PORT);



/*****************************************************************************/
/*                                    ODOM                                   */
/*****************************************************************************/

// imu bias

float imu_bias_x = 0;
float imu_bias_y = 0;
float imu_bias_z = 0;

void solve_imu_bias(int32_t timeout) {
	int32_t start_time = pros::millis();

	imu_bias_x = 0;
	imu_bias_y = 0;
	imu_bias_z = 0;

	int cycle_count = 0;
	while(pros::millis() <= start_time + timeout) {
		imu_bias_x += imu.get_accel().x;
		imu_bias_y += imu.get_accel().y;
		imu_bias_z += imu.get_accel().z;

		++cycle_count;

		pros::delay(PROCESS_DELAY);
	}

	imu_bias_x /= cycle_count;
	imu_bias_y /= cycle_count;
	imu_bias_z /= cycle_count;
}

// robot position
std::atomic<float> xPos (0);
std::atomic<float> yPos (0);
std::atomic<float> zPos (0);
std::atomic<float> dist (0);
std::atomic<float> heading (0);

int left_motors_prev_pos = 0;
int right_motors_prev_pos = 0;

long last_pos_update = 0;

void get_robot_position(long last_update) {
	last_pos_update = last_update;

	xPos.fetch_add(imu.get_accel().x - imu_bias_x);
	yPos.fetch_add(imu.get_accel().y - imu_bias_y);
	zPos.fetch_add(imu.get_accel().z - imu_bias_z);

	dist.fetch_add(((dt_left_motors.get_position() - left_motors_prev_pos) + (dt_right_motors.get_position() - right_motors_prev_pos)) / 2.0f);
	left_motors_prev_pos = dt_left_motors.get_position();
	right_motors_prev_pos = dt_right_motors.get_position();

	heading.store(imu.get_heading());
}



/*****************************************************************************/
/*                                COMPETITION                                */
/*****************************************************************************/

void initialize() {
	pros::lcd::initialize();

	imu.reset(true);
	solve_imu_bias(1000);

    pros::Task pos_tracking_task([&]() {
		last_pos_update = pros::millis();

        while (true) {
			get_robot_position(last_pos_update);
			
            pros::lcd::print(0, "xPos: %f", xPos.load());
            pros::lcd::print(1, "yPos: %f", yPos.load());
			pros::lcd::print(2, "dist: %f", dist.load());
            pros::lcd::print(3, "head: %f", heading.load());

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
		// Normalize current and target to [-180, 180]
		target = fmod((target + 180), 360) - 180;
		current = fmod((current + 180), 360) - 180;

		// Calculate the error (still in the range [-180, 180])
		float error = target - current;

		// Ensure the error is also in [-180, 180]
		error = fmod((error + 180), 360) - 180;
	
		return error;
		

	};

	// atomic instance of target heading cause that is that the PID process
	// requires
	std::atomic<float> atomic_target_heading (target_heading);
	// records the output of the PID process
	std::atomic<float> output (0);

	// start time of process
	int32_t start_time = pros::millis();

	// start the PID process
	pros::Task pid_process_task{[&] {
		pid_process(
				&heading,
				&atomic_target_heading,
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

	// remove task
	pid_process_task.remove();
}

void move_a_distance(float distance, int32_t timeout) {
	// atomic instance of target heading cause that is that the PID process
	// requires
	std::atomic<float> atomic_distance (distance);
	// records the output of the PID process
	std::atomic<float> output (0);

	// start time of process
	int32_t start_time = pros::millis();

	// start the PID process
	pros::Task pid_process_task{[&] {
		pid_process(
				&dist,
				&atomic_distance,
				timeout,
				&lateral_pid,
				&output
		);
	}};
	
	// apply the motor values
	while (pros::millis() < start_time + timeout) {
		dt_left_motors.move(output.load());
		dt_right_motors.move(output.load());

		// delay to save resources
		pros::delay(PROCESS_DELAY);
	}

	// brake motors at end of process
	dt_left_motors.brake();
	dt_right_motors.brake();

	// remove task
	pid_process_task.remove();
}

float prev_rotational_error = 0;

void pranav_norm_auton(std::atomic<float>& target_dist, std::atomic<float>& target_heading) {

}

void skills_auton_17pts(std::atomic<float>& target_dist, std::atomic<float>& target_heading) {
	mogo.set_value(true);
	target_dist.fetch_add(-500);
	
	intake.move(127);
	pros::delay(500);

	mogo.set_value(false);
	intake.brake();

	target_heading.store(0);

	target_dist.fetch_add(1100);
	pros::delay(1500);

	target_heading.store(90);
	pros::delay(1500);

	target_dist.fetch_add(-1250); 
	pros::delay(2000);

	mogo.set_value(true);
	target_heading.store(-5);
	pros::delay(1500);

	target_dist.fetch_add(1600);
	intake.move(127);
	pros::delay(3000);

	target_heading.store(-90);
	pros::delay(1650);

	target_dist.fetch_add(1500);
	pros::delay(3000);

	target_heading.store(-173);
	pros::delay(3000);

	target_dist.fetch_add(2250);
	pros::delay(3000);

	target_dist.fetch_add(-1500);
	pros::delay(1500);

	target_heading.store(-135);
	pros::delay(1500);

	target_dist.fetch_add(750);
	pros::delay(1000);

	target_heading.store(-90);
	pros::delay(1500);

	target_heading.store(35);
	pros::delay(2000);

	target_dist.fetch_add(-2500);
	pros::delay(3000);
}

void autonomous() {
	dt_left_motors.tare_position_all();
	dt_right_motors.tare_position_all();

	std::atomic<float> target_heading (0);
	auto normalize_rotation = [](float target, float current) {
		// Normalize current and target to the range [-180, 180]
		target = fmod((target + 180), 360) - 180;
		current = fmod((current + 180), 360) - 180;

		// Calculate the raw error
		float error = target - current;

		// Normalize the error to [-180, 180]
		error = fmod((error + 180), 360) - 180;

		float absErr = std::abs(error);
		float absPErr = std::abs(error + 360);
		float absSErr = std::abs(error - 360);

		if (absErr < absPErr && absErr < absSErr) return error;
		if (absPErr < absSErr && absPErr < absSErr) return error + 360;
		if (absSErr < absPErr && absSErr < absErr) return error - 360;
	};
	std::atomic<float> angular_output;
	pros::Task angular_pid_process_task{[&] {
		pid_process(
				&heading,
				&target_heading,
				120000,
				&angular_pid,
				&angular_output,
				normalize_rotation
		);
	}};

	std::atomic<float> target_dist (0);
	std::atomic<float> lateral_output;
	pros::Task lateral_pid_process_task{[&] {
		pid_process(
				&dist,
				&target_dist,
				120000,
				&lateral_pid,
				&lateral_output
		);
	}};

	pros::Task pid_output_manager_task([&]{
		while (true) {
			dt_left_motors.move(angular_output.load() + lateral_output.load());
			dt_right_motors.move(lateral_output.load() - angular_output.load());
		}
	});

	mogo.set_value(false);

	intake.brake();

	angular_pid_process_task.remove();
	lateral_pid_process_task.remove();
	pid_output_manager_task.remove();

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

		float drive_power = calcPowerCurve(
				drive_value, turn_value, drive_curve, DRIVE_RATIO, TURN_RATIO);
		float turn_power = calcPowerCurve(
				turn_value, drive_value, turn_curve, TURN_RATIO, DRIVE_RATIO);

		dt_left_motors.move(drive_power + turn_power);
		dt_right_motors.move(drive_power - turn_power);

		// intake
		if (master.get_digital(INTAKE_FWD_BUTTON))
			intake.move(intake_SPEED);
		else if (master.get_digital(INTAKE_REV_BUTTON))
			intake.move(-intake_SPEED);
		else intake.brake();
		
		// mogo
		if (master.get_digital(MOGO_ON_BUTTON))
			mogo.set_value(true);
		else if (master.get_digital(MOGO_OFF_BUTTON))
			mogo.set_value(false);

		// arm
		if (master.get_digital(ARM_UP_BUTTON))
			arm_target_pos += ARM_SPEED;
		else if (master.get_digital(ARM_DOWN_BUTTON))
			arm_target_pos -= ARM_SPEED;
		// constrain the target arm pos if we're not forcing it
		if (!master.get_digital(FORCE_ARM_POS_BUTTON)) {
			arm_target_pos.store(std::min(MAX_ARM_HEIGHT, std::max(MIN_ARM_HEIGHT,
					arm_target_pos.load()
			)));
		}
		// update current arm pos
		arm_pos.store(arm.get_position());
		// move arm to PID output
		arm.move(output.load());

		if (master.get_digital(RESET_ARM_POS_BUTTON)) arm.tare_position();

		// arm end
		if(master.get_digital(ARM_END_ON_BUTTON))
			arm_end.set_value(true);
		else if(master.get_digital(ARM_END_OFF_BUTTON))
			arm_end.set_value(false);

        pros::delay(PROCESS_DELAY);
    }
}