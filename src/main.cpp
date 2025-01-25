#include "main.h"
#include "pros/misc.h"

#include <cmath>
#include <limits>
#include <atomic>
#include <tuple>

#define wait(n) pros::delay(n)

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

float normalize_deg(float degree) {
    float normalized = std::fmod(degree, 360.0f);

    if (normalized > 180.0f) normalized -= 360.0f;
    else if (normalized < -180.0f) normalized += 360.0f;

    return normalized;
}

float deg_dif(float current, float target) {
    float diff = current - target;

    diff = std::fmod(diff + 180, 360);
    if (diff < 0) diff += 360;
    diff -= 180;

    return diff;
}



/*****************************************************************************/
/*                                   DATA                                    */
/*****************************************************************************/

struct Pose {
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> h;

	Pose(float x_pos, float y_pos, float head) {
		x.store(x_pos);
		y.store(y_pos);
		h.store(head);
	}

	auto operator()() {
        return std::tie(x, y, h);
    }
};

struct PIDController {
    float kp;
    float ki;
    float kd;
    float wind; // range of error for integral accumulation
	float clamp; // clamps the integral
	float decay; // applied when needed to smoothly increase or decrease a value
    float slew; // maximum acceleration and decelleration
	float small_error; // error range describing almost to target
	float large_error; // error range to start settling
	float tolerance; // acceptable error
};

struct DriveCurve {
    float deadband;
    float min_out;
    float expo_curve;
};



/*****************************************************************************/
/*                                  CONFIG                                   */
/*****************************************************************************/

constexpr long PROCESS_DELAY = 10;
constexpr long LONG_DELAY = 50;

// PORTS

constexpr int DT_FL_PORT = -15;
constexpr int DT_LM_PORT = 1;
constexpr int DT_BL_PORT = -20;

constexpr int DT_FR_PORT = 14;
constexpr int DT_MR_PORT = -18;
constexpr int DT_BR_PORT = 16;

constexpr int INTAKE_PORT = -2;

constexpr int DIDDY_PORT = 1;
constexpr int MOGO_PORT = 2;
constexpr int RAISE_INTAKE_PORT = 3;

constexpr int ARM_PORT = 5;

constexpr int IMU_PORT = 8;

// CONTROLS

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::CTRL_ANLG_LY;
constexpr pros::anlg_button TURN_JOYSTICK = pros::CTRL_ANLG_RX;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::CTRL_DIGI_L2;
constexpr pros::digi_button EJECT_RING_BUTTON = pros::CTRL_DIGI_X;

constexpr pros::digi_button DIDDY_TOGGLE_BUTTON = pros::CTRL_DIGI_X;

constexpr pros::digi_button MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::digi_button MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::digi_button ARM_UP_BUTTON = pros::CTRL_DIGI_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::CTRL_DIGI_R2;

constexpr pros::digi_button ARM_LOAD_POS_BUTTON = pros::CTRL_DIGI_UP;

// PID

PIDController lateral_pid (
		5, // kp
		0.7, // ki
		80, // kd
		5, // wind
		999, // clamp
		0, // decay
		999, // slew
		5, // small error
		12, // large error
		0.3 // tolerance
);

PIDController angular_pid (
		2, // kp
		0.2, // ki
		15, // kd
		10, // wind
		999, // clamp
		0, // decay
		999, // slew
		10, // small error
		30, // large error
		0.5 // tolerance
);

PIDController arm_pid (
		0.3, // kp
		0.05, // ki
		0.1, // kd
		100, // wind
		999, // clamp
		0.9, // decay
		999, // slew
		0, // small error
		50, // large error
		3 // tolerance
);

// AUTON

constexpr bool FORCE_AUTON = false; // if auton was not run, try again at start of op control

constexpr float DIST_MULTI = 35.5;

// DRIVE

constexpr int DRIVE_RATIO = 1;
constexpr int TURN_RATIO = 1;

constexpr float DRIVE_SLEW = 12;

constexpr bool SPEED_COMP = false;

constexpr int INTAKE_SPEED = 127;
constexpr int EJECT_BRAKE_CYCLES = 16;

constexpr float ARM_SPEED = 50;
constexpr float ARM_DOWN_SPEED_MULTI = 0.5;
constexpr float ARM_LOAD_POS = 250;

constexpr float ARM_BOTTOM_LIMIT = 20;
constexpr float ARM_TOP_LIMIT = 2000;

constexpr DriveCurve drive_lateral (3, 10, 3);
constexpr DriveCurve drive_angular (3, 10, 1);



/*****************************************************************************/
/*                                    PID                                    */
/*****************************************************************************/

struct PIDProcess {
    std::atomic<float>& value;
    std::atomic<float>& target;
    std::atomic<float>& output;
    const PIDController& pid;
    float max_speed;
    float min_speed;
    uint32_t life;
    std::function<float(float, float)> normalize_err;

    float prev_output = 0;
    float prev_error = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;

	float get_error() {
		if (normalize_err) return normalize_err(target.load(), value.load());
		else return target.load() - value.load();
	}

    PIDProcess(std::atomic<float>& value, std::atomic<float>& target, std::atomic<float>& output,
               const PIDController& pid, float max_speed, float min_speed, uint32_t life,
               std::function<float(float, float)> normalize_err = [](float err, float maxErr) { return err / maxErr; })
        : value(value), target(target), output(output), pid(pid), max_speed(max_speed),
          min_speed(min_speed), life(life), normalize_err(normalize_err) {}

    auto operator()() {
        return std::tie(value, target, output, pid, min_speed, max_speed, life, normalize_err,
                        prev_output, prev_error, error, integral, derivative);
    }
};


void pid_handle_process(PIDProcess& process) {

    auto [value, target, output, pid, min_speed, max_speed, life, normalize_err,
          prev_output, prev_error, error, integral, derivative] = process();

	printf("\nHANDLING PID PROCESS\n");
	printf("> STATE: value: %f, target: %f, output: %f\n", value.load(), target.load(), output.load());

    if (life <= 0) return;
    life -= 1;

    prev_output = output.load();
    prev_error = error;

    // error update
    error = target.load() - value.load();
	printf("> ERROR CALC: target(%f) - value(%f) = error(%f)\n", target.load(), value.load(), error);
    // normalize error if applicable
    if (normalize_err) {
		error = normalize_err(target.load(), value.load());
		printf("> CUST ERROR CALC: norm_error = error(%f)\n", error);
	}

    // reset integral if we crossed target
    if (sgn(prev_error) != sgn(error)) {
		integral = 0;
		printf("> RESET INTEGRAL\n");
	}

    // update integral if error in windup range
    if (std::abs(error) <= pid.wind) {
		integral += error;
		printf("> ACCUMULATE INTEGRAL: +%f = %f\n", error, integral);
	}
    // else decay integral
    else {
		integral *= pid.decay;
		printf("> DECAY INTEGRAL: *%f = %f\n", pid.decay, integral);
	}

    // clamp integral
    integral = std::min(pid.clamp, std::max(-pid.clamp, integral));
	printf("> CLAMP INTEGRAL: range(%f) = %f\n", pid.clamp, integral);
    
    // derivative update
    derivative = error - prev_error;
	printf("> UPDATE DERIV: error(%f) - prev_error(%f\n", error, prev_error);

    float real_error = error;
    float real_integral = integral;
    float real_derivative = derivative;

    // scale integral on small error
	printf("integral mult %f\n", (std::min(1.0f, std::abs(error) / pid.small_error)));
    real_integral *= (std::min(1.0f, std::abs(error) / pid.small_error));
    // scale derivative on large error
    printf("derivitave mult %f\n",  (1 - std::min(1.0f, std::abs(error) / pid.large_error)));
	real_derivative *= (1 - std::min(1.0f, std::abs(error) / pid.large_error));

	printf("err %f, int %f, der %f\n", real_error, real_integral, real_derivative);

    // calculate power
    float calc_power = real_error * pid.kp + real_integral * pid.ki + real_derivative * pid.kd;
	printf("> POWER CALC: %f\n", calc_power);
    // constrain power to slew
    calc_power = std::min(prev_output + pid.slew, std::max(prev_output - pid.slew, calc_power));
    printf("> SLEW CALC: %f\n", calc_power);
	// constrain power to min/max speed
    calc_power = std::min(max_speed, std::max(-max_speed, calc_power));
	printf("> MAX SPEED CALC: %f\n", calc_power);

    // set output power
    output.store(calc_power);
	printf("> OUTPUT: %f\n", calc_power);
}



/*****************************************************************************/
/*                                DRIVECURVE                                 */
/*****************************************************************************/

float drivecurve_calc_power(int value, int other_value, DriveCurve curve, int ratio, int other_ratio) {
	if (std::abs(value) <= curve.deadband) return 0;

	float ratio_mult = (float) ratio / (ratio + other_ratio);
	if (SPEED_COMP) {
		float dynamic_ratio = (
				(float) other_ratio * std::abs(other_value) / 127);
		ratio_mult = (float) ratio / (ratio + dynamic_ratio);
	}

	int sign_mult = sgn(value);

	float adj = 127 / std::pow(127, curve.expo_curve);
	float expo = std::pow(std::abs(value), curve.expo_curve);

	float arm_pos_output = (float) (adj * expo);
	
	return sign_mult * std::max(curve.min_out, arm_pos_output);
}



/*****************************************************************************/
/*                                  DEVICES                                  */
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

pros::IMU imu (IMU_PORT);



/*****************************************************************************/
/*                                    ODOM                                   */
/*****************************************************************************/

// imu bias

float imu_bias_x = 0;
float imu_bias_y = 0;
float imu_bias_z = 0;
float imu_bias_h = 0;

void solve_imu_bias(int32_t life) {

	imu_bias_x = 0;
	imu_bias_y = 0;
	imu_bias_z = 0;
	imu_bias_h = 0;

	float prev_heading = 0;

	int cycle_count = 0;
	while(life >= 0) {
		life -= PROCESS_DELAY;

		imu_bias_x += imu.get_accel().x;
		imu_bias_y += imu.get_accel().y;
		imu_bias_z += imu.get_accel().z;
		float curr_heading = imu.get_heading();
		imu_bias_h += curr_heading - prev_heading;
		prev_heading = curr_heading;

		++cycle_count;

		pros::delay(PROCESS_DELAY);
	}

	imu_bias_x /= cycle_count;
	imu_bias_y /= cycle_count;
	imu_bias_z /= cycle_count;
	imu_bias_h /= cycle_count;
}

// robot position
Pose robot_pose (0, 0, 0);
std::atomic<float> dist (0);
std::atomic<float> prev_dist(0);

std::atomic<float> arm_pos(0);

int left_motors_prev_pos = 0;
int right_motors_prev_pos = 0;

void get_robot_position() {
	auto [x_pos, y_pos, heading] = robot_pose();

	prev_dist.store(dist.load());

	dist.fetch_add(((dt_left_motors.get_position() - left_motors_prev_pos) + (dt_right_motors.get_position() - right_motors_prev_pos)) / 2.0f / DIST_MULTI);
	left_motors_prev_pos = dt_left_motors.get_position();
	right_motors_prev_pos = dt_right_motors.get_position();

	x_pos.fetch_add(std::cos(heading * M_PI / 180) * (dist.load() - prev_dist.load()));
	y_pos.fetch_add(std::sin(heading * M_PI / 180) * (dist.load() - prev_dist.load()));

	heading.store(normalize_deg(imu.get_heading()));

	arm_pos.store(arm.get_position());
}



/*****************************************************************************/
/*                                COMPETITION                                */
/*****************************************************************************/

bool init_done = false;

void init() {
	if (init_done) return;

	pros::lcd::initialize();

	imu.reset(true);
	solve_imu_bias(900);

    pros::Task pos_tracking_task([&]() {
		auto [x_pos, y_pos, heading] = robot_pose();

        while (true) {
			get_robot_position();
			
            pros::lcd::print(0, "x_pos: %f", x_pos.load());
            pros::lcd::print(1, "y_pos: %f", y_pos.load());
            pros::lcd::print(3, "head: %f", heading.load());

            pros::delay(PROCESS_DELAY);
        }
    });

	init_done = true;
}

void initialize() {
	init();
}

void competition_initialize() {
	init();
}



/*****************************************************************************/
/*                                   AUTON                                   */
/*****************************************************************************/

bool auton_ran = false;

int auton_cycle_count = 0;

Pose target_pose (0, 0, 0);

float deg_to_point(float x, float z) {
	float deg = normalize_deg((std::atan2(x, z) * 180 / M_PI));
	return deg;
}

void wait_stable(PIDProcess pid_process) {
	while (std::abs(pid_process.get_error()) > pid_process.pid.tolerance) {
		wait(PROCESS_DELAY);
	}
}

void autonomous() {
	if (auton_ran) return;

	auton_ran = true;

	dt_left_motors.tare_position_all();
	dt_right_motors.tare_position_all();

	auto [x_pos, y_pos, heading] = robot_pose();

	std::atomic<float> target_heading (0);
	std::atomic<float> angular_output;
	PIDProcess angular_pid_process (
			heading,
			target_heading,
			angular_output,
			angular_pid,
			127, // max speed
			0, // min speed
			120000, // life
			deg_dif
	);
	auto error_mod = [](float target, float current) {
		float error = target - current;
		return error;
	};
	std::atomic<float> target_dist (0);
	std::atomic<float> lateral_output (0);
	PIDProcess lateral_pid_process (
			dist,
			target_dist,
			lateral_output,
			lateral_pid,
			127, // max speed
			0, // min speed
			120000, // life
			error_mod
	);

	std::atomic<float> target_arm_pos (0);
	std::atomic<float> arm_pos_output;
	PIDProcess arm_pid_process (
			arm_pos,
			target_arm_pos,
			arm_pos_output,
			arm_pid,
			127, // max speed
			0, // min speed
			120000, // life
			error_mod
	);

	pros::Task auton_task{[&] {
		while (true) {
			int start = pros::millis();
			// ++auton_cycle_count;

			// float x_dif = target_pose.x - robot_pose.x;
			// float y_dif = target_pose.y - robot_pose.y;

			// float dist_to_target = std::sqrt(x_dif * x_dif + y_dif * y_dif);
			// float deg_to_target = normalize_deg(deg_to_point(x_dif, y_dif));

			// float deg_err = deg_dif(deg_to_target, robot_pose.h);
			// bool fwd = (std::abs(deg_err) <= 90);

			// dist_to_target *= (1 - ((int) (deg_err) % 90) / 90);
			// if (!fwd) dist_to_target *= -1;

			// target_heading.store(deg_to_target);
			// target_dist.store(dist.load() + dist_to_target);

			// pid

			pid_handle_process(angular_pid_process);
			pid_handle_process(lateral_pid_process);

			dt_left_motors.move(angular_output.load() + lateral_output.load());
			dt_right_motors.move(lateral_output.load() - angular_output.load());

			pid_handle_process(arm_pid_process);

			arm.move(arm_pos_output.load());

			wait(PROCESS_DELAY);
		}
	}};

	lateral_pid_process.max_speed = 80;
	target_dist.store(-31);
	wait(1500);

	mogo.set_value(true);
	wait(500);

	intake.move(INTAKE_SPEED);
	wait(800);

	target_heading.store(-125);
	wait(800);

	lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(21.13);
	wait(800);

	target_heading.store(-105);
	wait(800);

	target_dist.fetch_add(10);
	wait(800);

	target_heading.store(15);
	wait(800);

	target_dist.fetch_add(21.13);
	wait(1000);

	auton_task.remove();
}



/*****************************************************************************/
/*                                  DRIVING                                  */
/*****************************************************************************/

float prev_drive_power = 0;

void opcontrol() {
	if (FORCE_AUTON) autonomous();

	std::atomic<float> arm_target_pos = 0;
	std::atomic<float> arm_pos_output (0);
	// dampens the error when moving downward to prevent dropping the arm
	auto error_mod = [](float target, float current) {
		float error = target - current;
		return error;
	};
	PIDProcess arm_pid_process (
			arm_pos,
			arm_target_pos,
			arm_pos_output,
			arm_pid,
			127,
			0,
			1200000, // 20 min time cause max said so
			error_mod
	);

	int intake_brake_timer = 0;

    while (true) {
		// driving
        int drive_value = master.get_analog(DRIVE_JOYSTICK);
        int turn_value = master.get_analog(TURN_JOYSTICK);

		float drive_power = drivecurve_calc_power(
				drive_value, turn_value, drive_lateral, DRIVE_RATIO, TURN_RATIO);
		drive_power = std::min(prev_drive_power + DRIVE_SLEW, std::max(prev_drive_power - DRIVE_SLEW, drive_power));

		float turn_power = drivecurve_calc_power(
				turn_value, drive_value, drive_angular, TURN_RATIO, DRIVE_RATIO);

		dt_left_motors.move(drive_power + turn_power);
		dt_right_motors.move(drive_power - turn_power);

		prev_drive_power = drive_power;

		// intake
		if (master.get_digital(EJECT_RING_BUTTON) && intake_brake_timer <= 0) {
			intake_brake_timer = EJECT_BRAKE_CYCLES;
		}
		if (intake_brake_timer > 0) {
			intake.brake();
			--intake_brake_timer;
		} else {
			if (master.get_digital(INTAKE_FWD_BUTTON))
				intake.move(INTAKE_SPEED);
			else if (master.get_digital(INTAKE_REV_BUTTON))
				intake.move(-INTAKE_SPEED);
			else intake.brake();
		}
		
		// mogo
		if (master.get_digital(MOGO_ON_BUTTON))
			mogo.set_value(true);
		else if (master.get_digital(MOGO_OFF_BUTTON))
			mogo.set_value(false);

		printf("CYCLE\n");
		// arm
		if (master.get_digital(ARM_UP_BUTTON)) {
			printf("\tarm up\n");
			arm_target_pos += ARM_SPEED;
		}
		else if (master.get_digital(ARM_DOWN_BUTTON)) {
			printf("\tarm down\n");
			arm_target_pos -= ARM_SPEED;
		}
		printf("\tarm tp %f\n", arm_target_pos.load());
		// arm limiters
		if (arm_target_pos.load() < ARM_BOTTOM_LIMIT) arm_target_pos.store(ARM_BOTTOM_LIMIT);
		if (arm_target_pos.load() > ARM_TOP_LIMIT) arm_target_pos.store(ARM_TOP_LIMIT);
		printf("\tarm lim %f\n", arm_target_pos.load());
		// arm load macro
		if (master.get_digital(ARM_LOAD_POS_BUTTON)) {
			arm_target_pos = ARM_LOAD_POS;
			printf("\tarm macro %f\n", arm_target_pos.load());
		}
		// run pid
		pid_handle_process(arm_pid_process);
		// move arm to PID arm_pos_output
		arm.move(arm_pos_output.load());
		printf("\tarm out %f\n", arm_pos_output.load());
		printf("\tarm pos %f\n", arm_pos.load());

        pros::delay(PROCESS_DELAY);
    }
}