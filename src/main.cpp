#include "main.h"

// constants

const double EXP = std::pow(128, 1/127);
const double RADIAN = M_PI / 180;

constexpr int TILE_SIDE = 24;

constexpr int WHEEL_TICKS_PER_ROTATION = 900;

constexpr float WHEEL_DIAMETER = 2.75;
const float WHEEL_CIRCUM = M_PI * WHEEL_DIAMETER;

const int TICKS_PER_TILE = TILE_SIDE / WHEEL_CIRCUM * WHEEL_TICKS_PER_ROTATION;

// config

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

pros::adi::AnalogOut arm (ARM_PORT);

pros::IMU intertial_sensor (INTERTIAL_SENSOR_PORT);

void initialize() {
	// resets and calibrates the imu. Should only take a few seconds
	intertial_sensor.reset();

	// resets wheel encoder values.
	drive_train_left_motor_group.tare_position_all();
	drive_train_left_motor_group.tare_position_all();
	
	autonomous();
}

void disabled() {}

void competition_initialize() {}

// gets the robot position based on the previous position and wheel encoder values.
void get_position(double* prev_pos, double* prev_motor_pos) {
	prev_pos[2] = intertial_sensor.get_heading();

	double robot_move_dist = 
		((drive_train_left_motor_group.get_position(0) - prev_motor_pos[0])
		+ (drive_train_right_motor_group.get_position(0) - prev_motor_pos[1]))
		/ 2;

	prev_pos[0] += robot_move_dist * std::cos(prev_pos[2] * RADIAN);
	prev_pos[1] += robot_move_dist * std::sin(prev_pos[2] * RADIAN);
}

double robot_pos[3];
double motor_encoder_values[2];

bool match(int a, int b, int err) {
	return a > b - err && a < b + err;
}

void move(int ticks, int speed) {
	ticks *= -1;

	drive_train_left_motor_group.move(speed * (ticks < 0 ? -1 : 1));
	drive_train_right_motor_group.move(speed * (ticks < 0 ? -1 : 1));

	int target_ticks_left = drive_train_left_motor_group.get_position(0);
	int target_ticks_right = drive_train_right_motor_group.get_position(0);

	int left_pos;
	int right_pos;
	do {
		pros::delay(20);
		int left_pos = drive_train_left_motor_group.get_position(0);
		int right_pos = drive_train_right_motor_group.get_position(0);
	} while (!match(left_pos, target_ticks_left, 5) && !match(right_pos, target_ticks_right, 5));

	drive_train_left_motor_group.move(0);
	drive_train_right_motor_group.move(0);
}

void rotate(int ticks, int speed) {
	ticks *= -1;

	drive_train_left_motor_group.move(speed * (ticks < 0 ? -1 : 1));
	drive_train_right_motor_group.move(speed * (ticks < 0 ? 1 : -1));

	int target_ticks_left = drive_train_left_motor_group.get_position(0);
	int target_ticks_right = drive_train_right_motor_group.get_position(0);

	int left_pos;
	int right_pos;
	do {
		pros::delay(20);
		int left_pos = drive_train_left_motor_group.get_position(0);
		int right_pos = drive_train_right_motor_group.get_position(0);
	} while (!match(left_pos, target_ticks_left, 5) && !match(right_pos, target_ticks_right, 5));

	drive_train_left_motor_group.move(0);
	drive_train_right_motor_group.move(0);
}

void move_to(double x_pos, double y_pos) {

}

void rotate_to(double heading) {

}

void autonomous() {
	move(1000, 30);
}

void opcontrol() {
	master.set_text(0, 0, "op mode");

	while (true) {
		// basic drive controls
		int left_stick_y = master.get_analog(ANALOG_LEFT_Y);
		int right_stick_x = master.get_analog(ANALOG_RIGHT_X);

		float drive_speed = (0 < left_stick_y - left_stick_y < 0) * (std::pow(EXP, left_stick_y) - 1);
		float drive_turn_speed = right_stick_x * DRIVE_TRAIN_TURN_SENSITIVITY;

		drive_train_left_motor_group.move(drive_speed + drive_turn_speed);
		drive_train_right_motor_group.move(drive_speed - drive_turn_speed);

		// gets the position of the robot
		get_position(robot_pos, motor_encoder_values);

		// displays position of robot on screen
		// master.set_text(1, 1, "X Position");
		// master.set_text(2, 1, "Y Position");
		// master.set_text(3, 1, "Heading");

		master.set_text(0, 1, std::to_string(robot_pos[0]));
		master.set_text(1, 1, std::to_string(robot_pos[1]));
		master.set_text(2, 1, std::to_string(robot_pos[2]));

		pros::delay(20);
	}
}