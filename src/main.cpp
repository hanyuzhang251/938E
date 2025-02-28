

#include "main.h"
#include "pros/misc.h"
#include "pros/optical.hpp"

#include <cmath>
#include <limits>
#include <atomic>
#include <tuple>
#include <queue>
#include <cstring>
#include <cstdio>

inline void wait(uint32_t n) {
	pros::delay(n);
}

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

//                                                                             
struct Toggle {                                                                
	bool value;
	bool ptrigger = false;

	Toggle(bool value_ = false) : value(value_), ptrigger(false) {}

	void tick(bool trigger) {
		if (trigger && !ptrigger) value = !value;
		ptrigger = trigger;
	}
};

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
	float decay; // smoothly decreases integral
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

constexpr int V_TRACKING_WHEEL_PORT = 7;

constexpr int INTAKE_PORT = -2;

constexpr int DIDDY_PORT = -1;

constexpr int MOGO_PORT = 2;
constexpr int RAISE_INTAKE_PORT = 3;

constexpr int ARM_R_PORT = -5;
constexpr int ARM_L_PORT = 21;

constexpr int IMU_PORT = 8;

constexpr int OPTICAL_PORT = 6;

// CONTROLS

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::CTRL_ANLG_LY;
constexpr pros::anlg_button TURN_JOYSTICK = pros::CTRL_ANLG_RX;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::CTRL_DIGI_L2;
constexpr pros::digi_button EJECT_RING_BUTTON = pros::CTRL_DIGI_X;

constexpr pros::digi_button DIDDY_TOGGLE_BUTTON = pros::CTRL_DIGI_X;
Toggle diddy_toggle (true);

constexpr pros::digi_button MOGO_TOGGLE_BUTTON = pros::CTRL_DIGI_A;
Toggle mogo_toggle (false);

constexpr pros::digi_button ARM_UP_BUTTON = pros::CTRL_DIGI_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::CTRL_DIGI_R2;

constexpr pros::digi_button ARM_INCREMENT_UP = pros::CTRL_DIGI_UP;
constexpr pros::digi_button ARM_INCREMENT_DOWN = pros::CTRL_DIGI_DOWN;
bool arm_up_p = false, arm_down_p = false;

bool p_nav_toggle = false;
constexpr pros::digi_button MENU_TOGGLE = pros::CTRL_DIGI_Y;
bool p_menu_toggle = false;
constexpr pros::digi_button SETTING_TOGGLE = pros::CTRL_DIGI_A;
bool p_setting_toggle = false;

bool override_inputs = false;

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
		2.5, // kp
		0.1, // ki
		24, // kd
		30, // wind
		999, // clamp
		0, // decay
		999, // slew
		0, // small error
		999, // large error
		2.5 // tolerance
);

PIDController arm_pid (      
		0.3, // kp
		0.02, // ki
		0.5, // kd
		30, // wind
		999, // clamp
		0, // decay
		999, // slew
		0, // small error
		999, // large error
		0 // tolerance
);

// AUTON

constexpr bool FORCE_AUTON = false; // if auton was not run, try again at start of op control

constexpr float DIST_MULTI = 29;

bool intake_override = false;

bool racism = false; // true = red bad

bool color_sort = true;
constexpr int CS_OUTTAKE_DELAY = 75;
constexpr int OUTTAKE_TICKS = 500;

constexpr float RED_HUE = 5;
constexpr float BLUE_HUE = 210;

constexpr float HUE_TOLERANCE = 15;

constexpr float DIST_TOLERANCE = 125;

// DRIVE

constexpr int DRIVE_RATIO = 1;
constexpr int TURN_RATIO = 1;

constexpr float DRIVE_SLEW = 127;

constexpr bool SPEED_COMP = false;

constexpr int INTAKE_SPEED = 127;
constexpr int EJECT_BRAKE_CYCLES = 16;

constexpr float ARM_SPEED = 70;
constexpr float ARM_MAX_SPEED = 300;

constexpr float ARM_INCREMENT = 80;

constexpr float ARM_BOTTOM_LIMIT = 0;
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
        if (normalize_err) {
            return normalize_err(target.load(), value.load());
        } else {
            return target.load() - value.load();
        }
    }

    PIDProcess(
        std::atomic<float>& value,
        std::atomic<float>& target,
        std::atomic<float>& output,
        const PIDController& pid,
        float max_speed,
        float min_speed,
        uint32_t life,
        std::function<float(float, float)> normalize_err =
            [](float err, float maxErr) { return err / maxErr; })
        : value(value),
          target(target),
          output(output),
          pid(pid),
          max_speed(max_speed),
          min_speed(min_speed),
          life(life),
          normalize_err(normalize_err) {}

    auto operator()() {
        return std::tie(
            value, target, output, pid, min_speed, max_speed, life,
            normalize_err, prev_output, prev_error, error, integral,
            derivative);
    }
};


void pid_handle_process(PIDProcess& process) {                                 

    auto [value, target, output, pid, min_speed, max_speed, life, normalize_err,
          prev_output, prev_error, error, integral, derivative] = process();

    if (life <= 0) return;
    life -= 1;

    prev_output = output.load();
    prev_error = error;

    // error update
    error = target.load() - value.load();
    // normalize error if applicable
    if (normalize_err) {
		error = normalize_err(target.load(), value.load());
	}

    // reset integral if we crossed target                                     
    if (sgn(prev_error) != sgn(error)) {
		integral = 0;
	}

    // update integral if error in windup range
    if (std::abs(error) <= pid.wind) {
		integral += error;
	}
    // else decay integral
    else {
		integral *= pid.decay;
	}

    // clamp integral
    integral = std::min(pid.clamp, std::max(-pid.clamp, integral));
    
    // derivative update                                                       
    derivative = error - prev_error;

    float real_error = error;                                                  
    float real_integral = integral;
    float real_derivative = derivative;

    // scale integral on small error
    real_integral *= (std::min(
		1.0f,
		std::max(0.3f, std::abs(error) / pid.small_error))
	);
    // scale derivative on large error
	real_derivative *= (1 - std::min(1.0f, std::abs(error) / pid.large_error));

    // calculate power
    float calc_power =
		real_error * pid.kp
		+ real_integral * pid.ki
		+ real_derivative * pid.kd;
    // constrain power to slew
    calc_power = std::min(
		prev_output + pid.slew,
		std::max(prev_output - pid.slew, calc_power)
	);
	// constrain power to min/max speed
    calc_power = std::min(max_speed, std::max(-max_speed, calc_power));

    // set output power
    output.store(calc_power);
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

pros::Rotation v_tracking_wheel (V_TRACKING_WHEEL_PORT);

pros::Motor intake (INTAKE_PORT);

pros::adi::DigitalOut mogo (MOGO_PORT);

pros::adi::DigitalOut diddy (DIDDY_PORT);

pros::MotorGroup arm ({ARM_R_PORT, ARM_L_PORT});

pros::IMU imu (IMU_PORT);

pros::Optical optical (OPTICAL_PORT);

using Task = std::pair<std::function<void()>, uint32_t>;
auto task_cmp = [](auto a, auto b) {return a.second > b.second;};
std::priority_queue<Task, std::vector<Task>, decltype(task_cmp)> tasks;

void schedule_task(std::function<void()> func, uint32_t time, bool relative = true) {
	if (relative) time += pros::millis();

	tasks.emplace(func, time);
}

bool check_tasks() {
	bool task_run = false;

	while (!tasks.empty()) {
		Task top = tasks.top();
		if (pros::millis() >= top.second) {
			tasks.pop();
			top.first();
			task_run = true;
		} else {
			break;
		}
	}

	return task_run;
}

char ctrl_log[3][15];
int ctrl_log_ptr = 0;
uint32_t ctrl_log_update = pros::millis();



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
Pose robot_pose_mod(0, 0, 0);                                                  
Pose robot_pose (0, 0, 0);
Pose robot_ipose (0, 0, 0);
std::atomic<float> dist (0);
std::atomic<float> prev_dist(0);

std::atomic<float> arm_pos(0);

int left_motors_prev_pos = 0;
int right_motors_prev_pos = 0;//

uint32_t prev_v_tracking_wheel_pos = 0;

void get_robot_position() {
	prev_dist.store(dist.load());

	dist.fetch_add(
		((dt_left_motors.get_position() - left_motors_prev_pos)
		+ (dt_right_motors.get_position() - right_motors_prev_pos))
		/ 2.0f / DIST_MULTI);
	left_motors_prev_pos = dt_left_motors.get_position();
	right_motors_prev_pos = dt_right_motors.get_position();

	auto [x_ipos, y_ipos, iheading] = robot_ipose();

	x_ipos.fetch_add(
		std::cos(iheading * M_PI / 180)
		* (dist.load() - prev_dist.load()));
	y_ipos.fetch_add(
		std::sin(iheading * M_PI / 180)
		* (dist.load() - prev_dist.load()));

	iheading.store(normalize_deg(imu.get_heading()));

	auto [x_pos, y_pos, heading] = robot_pose();
	auto [x_mpos, y_mpos, mheading] = robot_pose_mod();

	x_pos.store(x_ipos + x_mpos);
	y_pos.store(y_ipos + y_mpos);
	heading.store(iheading + mheading);

	arm_pos.store(arm.get_position());
}



/*****************************************************************************/
/*                                COMPETITION                                */
/*****************************************************************************/

uint32_t run_start = pros::millis();

bool init_done = false;

std::atomic<bool> red_ring_seen = false;
std::atomic<bool> blue_ring_seen = false;

void init() {
	if (init_done) return;

	optical.set_integration_time(17);

	pros::lcd::initialize();
	master.clear();

	intake.set_current_limit(2700);
	

	imu.reset(true);
	solve_imu_bias(900);

    pros::Task pos_tracking_task([&]() {
		auto [x_pos, y_pos, heading] = robot_pose();
		auto [x_ipos, y_ipos, iheading] = robot_ipose();

		v_tracking_wheel.reset();//
		v_tracking_wheel.reset_position();//
		v_tracking_wheel.set_position(0);

        while (true) {
			get_robot_position();
			
            pros::lcd::print(0, "x_pos: %f", x_pos.load());
            pros::lcd::print(1, "y_pos: %f", y_pos.load());
            pros::lcd::print(2, "head:  %f", heading.load());

            pros::delay(PROCESS_DELAY);
        }
    });

	pros::Task async_tasks([&]() {
		bool ctrl_in_menu = false;
		int ctrl_menu_ptr = 0;

		while(true) {
			check_tasks();

			if (master.get_digital(MENU_TOGGLE) && !p_menu_toggle) {
				ctrl_in_menu = !ctrl_in_menu;
				override_inputs = ctrl_in_menu;
			}
			p_menu_toggle = master.get_digital(MENU_TOGGLE);

			if (ctrl_in_menu) {
				if (!p_nav_toggle && ctrl_menu_ptr > 0 && master.get_digital(pros::CTRL_DIGI_UP)) --ctrl_menu_ptr;
				if (!p_nav_toggle && ctrl_menu_ptr < 2 && master.get_digital(pros::CTRL_DIGI_DOWN)) ++ctrl_menu_ptr;
				p_nav_toggle = master.get_digital(pros::CTRL_DIGI_UP) || master.get_digital(pros::CTRL_DIGI_DOWN);
				
				if (master.get_digital(SETTING_TOGGLE) && !p_setting_toggle) {
					switch(ctrl_menu_ptr) {
						case 0: {
							racism = !racism;
							break;
						}
						case 1: {
							color_sort = !color_sort;
							break;
						}
						default: {}
					}
				}
				p_setting_toggle = master.get_digital(SETTING_TOGGLE);

				std::snprintf(ctrl_log[0], 15, "%ccolor: %s              ", ctrl_menu_ptr == 0 ? '>' : ' ', racism ? "BLUE" : "RED ");
				std::snprintf(ctrl_log[1], 15, "%ccolorsort: %s              ", ctrl_menu_ptr == 0 ? '>' : ' ', color_sort ? "ON " : "OFF");
				std::snprintf(ctrl_log[2], 15, "%couttake: %s              ", ctrl_menu_ptr == 0 ? '>' : ' ', color_sort ? "ON " : "OFF");
			} else {
				std::snprintf(ctrl_log[0], 15, "time: %ld               ", pros::millis() - run_start);
				std::snprintf(ctrl_log[1], 15, "");
				std::snprintf(ctrl_log[2], 15, "");
			}

			if (pros::millis() >= ctrl_log_update) {
				ctrl_log_update += 51;
				master.print(ctrl_log_ptr, 0, "%s", ctrl_log[ctrl_log_ptr]);
				
				ctrl_log_ptr++;
				if (ctrl_log_ptr >= 3) ctrl_log_ptr = 0;
			}

			wait(PROCESS_DELAY);
		}
	});

	optical.set_led_pwm(100);

	pros::Task intake_helper_task([&]() {
		int cs_outtake_delay = 0;
		int cs_outtake_ticks = 0;
		bool prev_verdict = false;

		while (true) {
			pros::delay(PROCESS_DELAY);

			if (!color_sort) continue;

			float r_hue_min = std::fmod(RED_HUE - HUE_TOLERANCE + 360, 360.0f);
			float r_hue_max = std::fmod(RED_HUE + HUE_TOLERANCE, 360.0f);
			float b_hue_min = std::fmod(BLUE_HUE - HUE_TOLERANCE + 360, 360.0f);
			float b_hue_max = std::fmod(BLUE_HUE + HUE_TOLERANCE, 360.0f);

			bool b_ring = false;
			if (b_hue_min <= b_hue_max) {
				// if it's normal, just check the ranges
				b_ring = b_hue_min <= optical.get_hue() && optical.get_hue() <= b_hue_max;
			} else if (b_hue_min >= b_hue_max) {
				// if the range goes around 0, say hue min is 330 while hue_max is 30,
				// 0 and 360 are used as bounds.
				b_ring = optical.get_hue() <= b_hue_max || optical.get_hue() >= b_hue_min;
			}
			if (b_ring) blue_ring_seen.store(true);

			bool r_ring = false;
			if (r_hue_min <= r_hue_max) {
				// if it's normal, just check the ranges
				r_ring = r_hue_min <= optical.get_hue() && optical.get_hue() <= r_hue_max;
			} else if (r_hue_min >= r_hue_max) {
				// if the range goes around 0, say hue min is 330 while hue_max is 30,
				// 0 and 360 are used as bounds.
				r_ring = optical.get_hue() <= r_hue_max || optical.get_hue() >= r_hue_min;
			}
			if (r_ring) red_ring_seen.store(true);

			bool outtake = racism && r_ring || !racism && b_ring;

			bool verdict = outtake && optical.get_proximity() >= DIST_TOLERANCE;
			if (verdict) {
				cs_outtake_ticks = OUTTAKE_TICKS / PROCESS_DELAY;
				if (!prev_verdict) {
					cs_outtake_delay = CS_OUTTAKE_DELAY / PROCESS_DELAY;
				}
			}

			// expected that intake_override is followed appropriately
			if (cs_outtake_delay > 0) {
				--cs_outtake_delay;
			} else if (cs_outtake_ticks > 0) {
				intake_override = true;
				intake.move(-INTAKE_SPEED);
				--cs_outtake_ticks;
			} else {
				intake_override = false;
			}

			prev_verdict = verdict;
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

struct Path {
	// std::pair<float, float>
};

// std::queue<

bool lateral_movement = true;

Pose target_pose (0, 0, 0);

float deg_to_point(float x, float y) {
	float deg = normalize_deg(normalize_deg((std::atan2(x, y) * 180 / M_PI)) - 90);
	return deg;
}

void move_to_point(float x, float y, bool fwd_) {
	auto [tx_pos, ty_pos, fwd] = target_pose();
	tx_pos.store(x);
	ty_pos.store(y);
	fwd.store(fwd_);
}

void wait_cross(PIDProcess pid_process, float point, bool relative = true, int buffer_ticks = 0) {
	if (relative) point += pid_process.value.load();

	bool side = pid_process.value.load() >= point;
	while ((pid_process.value.load() >= point) == side) {
		wait(PROCESS_DELAY);
	}
	for (int i = 0; i < buffer_ticks; ++i) wait(PROCESS_DELAY);
}

void wait_stable(PIDProcess pid_process, uint32_t timeout = 5000, int buffer_ticks = 3, int min_stable_ticks = 8) {
	int stable_ticks = 0;
	uint32_t end_time = pros::millis() + timeout;

	while (stable_ticks < min_stable_ticks) {
		pros::lcd::print(4, "error: %f", pid_process.get_error());
		if (std::abs(pid_process.get_error()) <= pid_process.pid.tolerance) ++stable_ticks;
		else stable_ticks = 0;

		if (pros::millis() >= end_time) return;
		wait(PROCESS_DELAY);
	}
	for (int i = 0; i < buffer_ticks; ++i) wait(PROCESS_DELAY);
}

// flag bits 
void wait_for_ring(bool red = true, bool blue = false, uint32_t timeout = 3000) {
	if (red) red_ring_seen.store(false);
	if (blue) blue_ring_seen.store(false);

	uint32_t end_time = pros::millis() + timeout;

	while (!(red && red_ring_seen || blue && blue_ring_seen)) {
		if (pros::millis() >= end_time) return;

		wait(PROCESS_DELAY);
	}
}

void tap_ring(int n_times, int delay = 150, std::atomic<bool>* crashout = nullptr) {
	pros::Task async_task([&]() {
		for (int i = 0; i < n_times; ++i) {
			intake.brake();
			wait(delay);
			intake.move(INTAKE_SPEED);
			wait(delay);
			if (crashout && crashout->load()) break;
		}
	});
}

float t = 0;

void autonomous() {
	if (auton_ran) return;

	auton_ran = true;

	arm.tare_position();
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

	bool run_intake = false;
	int intake_stuck_ticks = 0;
	const int INTAKE_STUCK_LIMIT = 12;
	int intake_outtake_ticks = 0;
	const int INTAKE_OUTTAKE_DURATION = 15;


	pros::Task auton_task{[&] {
		float mtp_locked_heading = 0;
		bool mtp_heading_is_locked = false;

		while (true) {
			if (run_intake && intake.get_efficiency() < 3) {
				++intake_stuck_ticks;
			} else {
				intake_stuck_ticks = 0;
			}
			if (intake_stuck_ticks >= INTAKE_STUCK_LIMIT) {
				intake_outtake_ticks = INTAKE_OUTTAKE_DURATION;
			}
			if (intake_outtake_ticks > 0) {
				if (!intake_override) intake.move(-INTAKE_SPEED);
				--intake_outtake_ticks;
			} else {
				if (!intake_override) {
					if (run_intake) intake.move(INTAKE_SPEED);
					else intake.brake();
				}
			}

			int start = pros::millis();

			// if (mtp) {
			// 	float x_dif = target_pose.x - robot_pose.x;
			// 	float y_dif = target_pose.y - robot_pose.y;

			// 	float dist_to_target = std::sqrt(x_dif * x_dif + y_dif * y_dif);
			// 	float deg_to_target = normalize_deg(deg_to_point(x_dif, y_dif));
			// 	if (target_pose.h.load() == false) deg_to_target *= -1;

			// 	float deg_err = deg_dif(deg_to_target, robot_pose.h);
			// 	bool fwd = (std::abs(deg_err) <= 90);

			// 	dist_to_target *= (1 - ((int) (deg_err) % 90) / 90);
			// 	if (!fwd) dist_to_target *= -1;

			// 	// if we're at the target, stop the robot
			// 	if (std::abs(dist_to_target) <= lateral_pid.tolerance) {
			// 		target_heading.store(heading);
			// 		target_dist.store(dist.load());
			// 	} else {
			// 		target_heading.store(deg_to_target);
			// 		target_dist.store(dist.load() + dist_to_target);
			// 	}
			// }

			// pid

			pid_handle_process(angular_pid_process);
			if (lateral_movement) pid_handle_process(lateral_pid_process);

			if (lateral_movement) dt_left_motors.move(angular_output.load() + lateral_output.load());
			if (lateral_movement) dt_right_motors.move(lateral_output.load() - angular_output.load());

			pid_handle_process(arm_pid_process);

			arm.move(arm_pos_output.load());

			printf("target_dist %f\n", target_dist.load());\
			printf("dist %f\n", dist.load());

			wait(PROCESS_DELAY);
		}
	}};

<<<<<<< HEAD
	lateral_pid_process.max_speed = 90;
	target_dist.store(-40);
	wait(1500);

	lateral_movement = false;
	dt_left_motors.brake();
	dt_right_motors.brake();

	mogo.set_value(true);
	wait(500);

	intake.move(INTAKE_SPEED);
	wait(800);

	lateral_movement = true;

	target_heading.store(100);
	wait(800);

	lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(30);
	wait(1500);

	mogo.set_value(false);
	wait(500);

	target_heading.store(-60);
	wait(800);

	target_dist.fetch_add(70);
	wait(1500);

	intake.move(INTAKE_SPEED-INTAKE_SPEED);
	wait(300);

	target_heading.store(45);
	wait(800);

	target_dist.fetch_add(-42);
	wait(1000);

	mogo.set_value(true);
	wait(200);

	intake.move(INTAKE_SPEED);
	wait(500);

	target_heading.store(90);
	wait(500);

	target_arm_pos.store(3 * ARM_LOAD_POS);
	target_dist.fetch_add(8);

	wait(2000);
=======
	run_intake = true;
	wait(200);
	run_intake = false;;

	target_dist.fetch_add(16);
	wait_stable(lateral_pid_process);
	target_heading.store(90);
	wait_stable(angular_pid_process);
	lateral_pid_process.max_speed = 90;
	target_dist.fetch_add(-26);
	wait_cross(lateral_pid_process, -24);
	mogo.set_value(true);
	wait(250);
	lateral_pid_process.max_speed = 127;

	target_heading.store(0);
	wait_stable(angular_pid_process);

	run_intake = true;

	t = lateral_pid_process.value.load();
	lateral_pid_process.max_speed = 100;
	target_dist.fetch_add(87);
	wait_cross(lateral_pid_process, t + 12, false);
	target_heading.store(-40);
	wait_cross(lateral_pid_process, t + 24 + 25, false);
	target_heading.store(0);
	wait_cross(lateral_pid_process, t + 24 + 34 + 16, false);
	wait_stable(lateral_pid_process);

	lateral_pid_process.max_speed = 127;
	wait(600);
	target_heading.store(182);
	wait_stable(angular_pid_process);
	target_dist.fetch_add(77);////
	wait_cross(lateral_pid_process, 4);
	for (int i = 0; i < 16; ++i) {
		lateral_pid_process.max_speed = 127 - i * 6;
		wait(60);
	}

	wait_stable(lateral_pid_process, 2500);
	wait(600);
	
	target_dist.fetch_add(26);
	wait_stable(lateral_pid_process, 1000);
//f
	target_heading.store(-50);
	wait_stable(angular_pid_process);

	run_intake = true;

	lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(29);
	wait_cross(lateral_pid_process, 4.2);
	target_heading.store(0);
	wait_stable(lateral_pid_process, 2000);
	wait(500);
	target_dist.fetch_add(-34);
	wait_cross(lateral_pid_process, -10);
	target_heading.store(20);
	wait_stable(lateral_pid_process, 1250);

	mogo.set_value(false);
	wait(250);
	run_intake = false;//

	lateral_pid_process.max_speed = 127;

	target_heading.store(90);
	target_dist.fetch_add(24);
	wait_cross(lateral_pid_process, 12);
	target_heading.store(0);
	wait_stable(angular_pid_process, 500);
	target_dist.fetch_add(-24);
	wait_stable(lateral_pid_process, 1000);

	for (int i = 0; i < 3; ++i) {
		robot_pose.x.store(0);
		wait(PROCESS_DELAY);
	}
	imu.set_heading(0);

	target_dist.fetch_add(70);
	wait_cross(lateral_pid_process, 2.5);
	target_heading.store(90);
	
	
	wait_stable(lateral_pid_process);

	target_heading.store(-90);
	wait_stable(angular_pid_process);
	target_dist.fetch_add(-24);//
	wait_stable(lateral_pid_process);
	mogo.set_value(true);
	wait(250);

	target_heading.store(-3);
	wait_stable(angular_pid_process);

	run_intake = true;

	t = lateral_pid_process.value.load();
	lateral_pid_process.max_speed = 100;
	target_dist.fetch_add(90);
	wait_cross(lateral_pid_process, t + 12, false);
	target_heading.store(40);
	wait_cross(lateral_pid_process, t + 24 + 25, false);
	target_heading.store(-5);
	wait_cross(lateral_pid_process, t + 24 + 34 + 16, false);
	wait_stable(lateral_pid_process);

	lateral_pid_process.max_speed = 127;
	wait(600);
	target_heading.store(182);
	wait_stable(angular_pid_process);
	target_dist.fetch_add(80);////
	wait_cross(lateral_pid_process, 4);
	for (int i = 0; i < 17; ++i) {
		lateral_pid_process.max_speed = 127 - i * 6;
		wait(50);
	}

	wait_stable(lateral_pid_process, 2500);
	wait(600);

	target_dist.fetch_add(26);
	wait_stable(lateral_pid_process, 1000);
//f
	target_heading.store(50);
	wait_stable(angular_pid_process);

	run_intake = true;

lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(29);
	wait_cross(lateral_pid_process, 4.2);
	target_heading.store(0);
	wait_stable(lateral_pid_process, 2000);
	wait(500);

	target_dist.fetch_add(-34);
	wait_cross(lateral_pid_process, -10);
	target_heading.store(-30);
	wait_stable(lateral_pid_process, 1250);

	mogo.set_value(false);
	wait(250);
	run_intake = false;//

	target_heading.store(-90);
	target_dist.fetch_add(24);
	wait_cross(lateral_pid_process, 12);
	target_heading.store(0);
	wait_stable(angular_pid_process, 500);
	target_dist.fetch_add(-24);
	wait_stable(lateral_pid_process, 1000);

	for (int i = 0; i < 3; ++i) {
		robot_pose.x.store(0);
		wait(PROCESS_DELAY);
	}
	imu.set_heading(0);

	target_dist.fetch_add(130);
	wait_cross(lateral_pid_process, 6);
	target_heading.store(-45);
	wait_cross(lateral_pid_process, 34);
	for (int i = 0; i < 17; ++i) {
		lateral_pid_process.max_speed = 127 - i * 6;
		wait(65);
	}
	run_intake = true;
	wait_stable(lateral_pid_process);
	run_intake = false;
	target_heading.store(-135);
	wait_stable(angular_pid_process);
	lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(-36);
	for (int i = 0; i < 10; --i) {
		lateral_pid_process.max_speed = 127 - i * 5;
		wait(50);
	}
	wait_stable(lateral_pid_process);
	mogo.set_value(true);
	wait(250);
	run_intake = true;

lateral_pid_process.max_speed = 127;
	target_heading.store(-85);
	wait_stable(angular_pid_process);
	target_dist.fetch_add(48);
	for (int i = 0; i < 16; ++i) {
		lateral_pid_process.max_speed = 127 - i * 6;
		wait(50);
	}
	wait_stable(lateral_pid_process);

	target_heading.store(0);
	wait_stable(angular_pid_process);
	target_dist.fetch_add(10);
	wait_stable(lateral_pid_process, 1000);

	target_heading.store(95);
	wait_stable(angular_pid_process, 1000);
	lateral_pid_process.max_speed = 127;
	target_dist.fetch_add(-20);
	wait_stable(lateral_pid_process, 2000);
	mogo.set_value(false);
	wait(250);

	lateral_pid_process.max_speed = 90;
	target_heading.store(-100);
	target_dist.fetch_add(48);
	run_intake = true;
	wait(2300);
	run_intake = false;
	wait(500);

	target_heading.store(-87);
	target_dist.fetch_add(-80);
	wait_stable(lateral_pid_process);

	target_heading.store(180);
	wait_stable(angular_pid_process);

	target_dist.fetch_add(-24);
	wait_stable(lateral_pid_process, 1000);
	target_dist.fetch_add(1.5);
	wait(250);
	run_intake = true;
	wait(250);
	run_intake = false;

	target_heading.store(90);
	target_dist.fetch_add(72);
	
	wait_stable(lateral_pid_process);
	wait_stable(angular_pid_process);//
>>>>>>> main

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
	int32_t max_arm_current_draw = 0;

	bool run_intake = false;
	bool intake_dir = true;
	int intake_stuck_ticks = 0;
	const int INTAKE_STUCK_LIMIT = 8;
	int intake_outtake_ticks = 0;
	const int INTAKE_OUTTAKE_DURATION = 10;


    while (true) {
		// if (run_intake && intake.get_efficiency() < 3) {
		// 	++intake_stuck_ticks;
		// } else {
		// 	intake_stuck_ticks = 0;
		// }
		// if (intake_stuck_ticks >= INTAKE_STUCK_LIMIT) {
		// 	intake_outtake_ticks = INTAKE_OUTTAKE_DURATION;
		// }
		// if (intake_outtake_ticks > 0) {
		// 	intake.move(-INTAKE_SPEED);
		// 	--intake_outtake_ticks;
		// } else {
			if (!intake_override) {
				if (run_intake) intake.move((intake_dir ? 1 : -1) * INTAKE_SPEED);
				else intake.brake();
			}
		// }

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
		if (!intake_override) {
			if (!override_inputs && master.get_digital(INTAKE_FWD_BUTTON)) {
				run_intake = true;
				intake_dir = true;
			} else if (!override_inputs && master.get_digital(INTAKE_REV_BUTTON)){
				run_intake = true;
				intake_dir = false;
			} else run_intake = false;
		} else {
			// do nothing buh
		}
		
		// mogo
		if (!override_inputs) mogo_toggle.tick(master.get_digital(MOGO_TOGGLE_BUTTON));
		mogo.set_value(mogo_toggle.value);

		// diddy
		if (!override_inputs) diddy_toggle.tick(master.get_digital(DIDDY_TOGGLE_BUTTON));
		diddy.set_value(diddy_toggle.value);

		// arm
		bool arm_moved = false;
		float real_arm_speed = ARM_SPEED;
		if (!override_inputs && master.get_digital(ARM_UP_BUTTON)) {
			arm_target_pos += real_arm_speed;
			arm_moved = true;
		}
		else if (!override_inputs && master.get_digital(ARM_DOWN_BUTTON)) {
			arm_target_pos -= real_arm_speed;
			arm_moved = true;
		}//

		// arm increment

		if (!override_inputs && master.get_digital(ARM_INCREMENT_UP) && !arm_up_p) {
			arm_target_pos.store(arm_pos.load() + ARM_INCREMENT);
		}
		arm_up_p = master.get_digital(ARM_INCREMENT_UP);

		// if (!override_inputs && master.get_digital(ARM_INCREMENT_DOWN) && !arm_down_p) {
		// 	arm_target_pos.store(arm_pos.load() - ARM_INCREMENT);
		// }
		// arm_down_p = master.get_digital(ARM_INCREMENT_DOWN);

		if (std::abs(arm_pid_process.error) > ARM_MAX_SPEED) {
			arm_target_pos = arm_pos.load() + sgn(arm_pid_process.error) * ARM_MAX_SPEED;
		}

		// run pid
		pid_handle_process(arm_pid_process);
		// move arm to PID arm_pos_output
		arm.move(arm_pos_output.load());

		if (arm.get_current_draw() > max_arm_current_draw)
		max_arm_current_draw = arm.get_current_draw();

		pros::lcd::print(5, "armpos %f", arm_target_pos.load());

        pros::delay(PROCESS_DELAY);
    }
}