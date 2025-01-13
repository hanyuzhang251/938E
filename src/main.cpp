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

float deg_dif(float angle1, float angle2) {
    float diff = angle1 - angle2;

    diff = std::fmod(diff + 180, 360);
    if (diff < 0) diff += 360;
    diff -= 180;

    return normalize_deg(std::abs(diff));
}



/*****************************************************************************/
/*                                   DATA                                    */
/*****************************************************************************/

struct Pose {
    float x;
    float y;
    float h;
};

struct PIDController {
    float kp;
    float ki;
    float kd;
    float wind; // range of error for integral accumulation
	float clamp; // clamps the integral
	float decay; // applied when needed to smoothly increase or decrease a value
    float slew; // maximum acceleration and decelleration
	float small_error; // range of acceptable error give or take a bit
	float large_error; // range of error for robot to start settling
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

constexpr int MOGO_PORT = 1;

constexpr int ARM_PORT = 5;
constexpr int ARM_END_PORT = 8;

constexpr int IMU_PORT = 8;

// CONTROLS

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::CTRL_ANLG_LY;
constexpr pros::anlg_button TURN_JOYSTICK = pros::CTRL_ANLG_RX;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::CTRL_DIGI_L2;
constexpr pros::digi_button EJECT_RING_BUTTON = pros::CTRL_DIGI_X;

constexpr pros::digi_button MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::digi_button MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::digi_button ARM_UP_BUTTON = pros::CTRL_DIGI_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::CTRL_DIGI_R2;

constexpr pros::digi_button ARM_LOAD_POS_BUTTON = pros::CTRL_DIGI_UP;

// PID

constexpr PIDController lateral_pid (
		0.045, // kp
		0.003, // ki
		0.035, // kd
		300, // wind
		999, // clamp
		0.9, // decay
		15, // slew
		100, // small error
		500 // large error
);

constexpr PIDController angular_pid (
		0.8, // kp
		0.1, // ki
		1, // kd
		30, // wind
		999, // clamp
		0.9, // decay
		999, // slew
		3, // small error
		500 // large error
);

constexpr PIDController arm_pid (
		0.6, // kp
		0.02, // ki
		0, // kd
		90, // wind
		999, // clamp
		0.9, // decay
		999, // slew
		3, // small error
		8 // large error
);

// AUTON

constexpr float DIST_MULTI = 35.5;

// DRIVE

constexpr int DRIVE_RATIO = 1;
constexpr int TURN_RATIO = 1;

constexpr bool SPEED_COMP = false;

constexpr int INTAKE_SPEED = 127;
constexpr int EJECT_BRAKE_CYCLES = 16;

constexpr float ARM_SPEED = 50;
constexpr float ARM_DOWN_SPEED_MULTI = 0.5;
constexpr float ARM_LOAD_POS = 215;

constexpr float ARM_BOTTOM_LIMIT = 50;
constexpr float ARM_TOP_LIMIT = 1900;

constexpr DriveCurve lateral (3, 10, 3);
constexpr DriveCurve angular (3, 10, 3);



/*****************************************************************************/
/*                                    PID                                    */
/*****************************************************************************/

struct PIDProcess {
	std::atomic<float>* value;
	std::atomic<float>* target;
	std::atomic<float>* output;
	const PIDController* pid;
	int32_t life;
	std::function<float(float, float)> normalize_err;

	float prev_output = 0;
	float prev_error = 0;
	float error = 0;
	float integral = 0;
	float derivative = 0;

	PIDProcess(const std::function<float(float, float)> normalize_err = nullptr)
			: value(value), target(target), output(output), pid(pid), life(life), normalize_err(normalize_err) {};

	auto operator()() {
        return std::tie(value, target, output, pid, life, normalize_err,
				prev_output, prev_error, error, integral, derivative);
    }
};

void pid_handle_process(PIDProcess& process) {
	auto [value, target, output, pid, life, normalize_err,
			prev_output, prev_error, error, integral, derivative] = process();

	if (life <= 0) return;

	life -= 1;

	prev_output = output->load();
	prev_error = error;

	// error update
	error = target->load() - value->load();
	// normalize error if applicable
	if (normalize_err) error = normalize_err(target->load(), value->load());

	// reset integral if we crossed target
	if (sgn(prev_error) != sgn(error)) integral = 0;

	// update integral if error in windup range
	if (std::abs(error) <= pid->wind) integral += error;
	// else decay integral
	else integral *= pid->decay;

	// clamp integral
	integral = std::min(pid->clamp, std::max(-pid->clamp, integral));
	
	// derivative update
	derivative = error - prev_error;

	float real_error = error;
	float real_integral = integral;
	float real_derivative = derivative;

	// scale integral on small error
	real_integral *= (1 - std::min(1.0f, std::abs(error) / pid->small_error));
	// scale derivative on large error
	real_derivative *= (1 - std::min(1.0f, std::abs(error) / pid->large_error));

	// calulate power
	float calc_power = real_error * pid->kp + integral * pid->ki + derivative * pid->kd;

	// constrain power to slew
	calc_power = std::min(prev_output + pid->slew, std::max(prev_output - pid->slew, calc_power));

	// set output power
	output->store(calc_power);
}