#include "main.h"

/**
 * CONSTANTS
 */

const double EXP = std::pow(128, 1/127);

/**
 * CONFIG
 */

constexpr float WHEEL_DIAMETER_INCHES = 2.5;

constexpr int DRIVE_TRAIN_LEFT_FRONT_MOTOR_PORT = 15;
constexpr int DIRVE_TRAIN_LEFT_MIDDLE_MOTOR_PORT = 1;
constexpr int DRIVE_TRAIN_LEFT_BACK_MOTOR_PORT = 4;

constexpr int DRIVE_TRAIN_RIGHT_FRONT_MOTOR_PORT = -14;
constexpr int DRIVE_TRAIN_RIGHT_MIDDLE_MOTOR_PORT = -18;
constexpr int DRIVE_TRAIN_RIGHT_BACK_MOTOR_PORT = -16;

constexpr int INTAKE_BOTTOM_PORT = 5;
constexpr int INTAKE_TOP_PORT = -2;

constexpr char MOGO_CLAMP_PORT = 'A';
constexpr char MOGO_BAR_PORT = 'B';

constexpr char ARM_PORT = 'D';

constexpr int INTERTIAL_SENSOR_PORT = 0;

constexpr float DRIVE_TRAIN_TURN_SENSITIVITY = 0.3f;

/**
 * COMPONENTS
 */

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor drive_train_left_front_motor (DRIVE_TRAIN_LEFT_FRONT_MOTOR_PORT);
pros::Motor drive_train_left_middle_motor (DIRVE_TRAIN_LEFT_MIDDLE_MOTOR_PORT);
pros::Motor drive_train_left_back_motor (DRIVE_TRAIN_LEFT_BACK_MOTOR_PORT);

pros::MotorGroup drive_train_left_motor_group ({
	DRIVE_TRAIN_LEFT_FRONT_MOTOR_PORT,
	DIRVE_TRAIN_LEFT_MIDDLE_MOTOR_PORT,
	DRIVE_TRAIN_LEFT_BACK_MOTOR_PORT
});

pros::Motor drive_train_right_front_motor (DRIVE_TRAIN_RIGHT_FRONT_MOTOR_PORT);
pros::Motor drive_train_right_middle_motor (DRIVE_TRAIN_RIGHT_MIDDLE_MOTOR_PORT);
pros::Motor drive_train_right_back_motor (DRIVE_TRAIN_RIGHT_BACK_MOTOR_PORT);

pros::MotorGroup drive_train_right_motor_group ({
	DRIVE_TRAIN_RIGHT_FRONT_MOTOR_PORT,
	DRIVE_TRAIN_RIGHT_MIDDLE_MOTOR_PORT,
	DRIVE_TRAIN_RIGHT_BACK_MOTOR_PORT
});

pros::Motor intake_bottom_motor (INTAKE_BOTTOM_PORT);
pros::Motor intake_top_motor (INTAKE_TOP_PORT);

pros::adi::AnalogOut mogo_clamp_piston (MOGO_CLAMP_PORT);
pros::adi::AnalogOut mogo_bar_piston (MOGO_BAR_PORT);

pros::IMU intertial_sensor (INTERTIAL_SENSOR_PORT);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	intertial_sensor.reset();

	drive_train_left_motor_group.tare_position_all();
	drive_train_left_motor_group.tare_position_all();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	while(true) {

	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		int left_stick_y = master.get_analog(ANALOG_LEFT_Y);
		int right_stick_x = master.get_analog(ANALOG_RIGHT_X);

		float drive_speed = (0 < left_stick_y - left_stick_y < 0) * (std::pow(EXP, left_stick_y) - 1);
		float drive_turn_speed = right_stick_x * DRIVE_TRAIN_TURN_SENSITIVITY;

		drive_train_left_motor_group.move(drive_speed + drive_turn_speed);
		drive_train_right_motor_group.move(drive_speed - drive_turn_speed);

		pros::delay(20);
	}
}