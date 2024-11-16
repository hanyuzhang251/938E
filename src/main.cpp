#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <cstddef>

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

constexpr int IMU_PORT = 0;

constexpr float DRIVE_TRAIN_TURN_SENSITIVITY = 0.3f;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup drive_train_left_motor_group ({
	DRIVE_TRAIN_LEFT_FRONT_MOTOR_PORT,
	DIRVE_TRAIN_LEFT_MIDDLE_MOTOR_PORT,
	DRIVE_TRAIN_LEFT_BACK_MOTOR_PORT
}, pros::MotorGearset::blue);

pros::MotorGroup drive_train_right_motor_group ({
	DRIVE_TRAIN_RIGHT_FRONT_MOTOR_PORT,
	DRIVE_TRAIN_RIGHT_MIDDLE_MOTOR_PORT,
	DRIVE_TRAIN_RIGHT_BACK_MOTOR_PORT
}, pros::MotorGearset::blue);

pros::Motor intake_bottom_motor (INTAKE_BOTTOM_PORT);
pros::Motor intake_top_motor (INTAKE_TOP_PORT);

pros::adi::AnalogOut mogo_clamp_piston (MOGO_CLAMP_PORT);
pros::adi::AnalogOut mogo_bar_piston (MOGO_BAR_PORT);

pros::adi::AnalogOut arm (ARM_PORT);

pros::IMU imu (IMU_PORT);

lemlib::Drivetrain drivetrain(&drive_train_left_motor_group, &drive_train_right_motor_group, 12.5, lemlib::Omniwheel::NEW_275, 450, 2);

lemlib::OdomSensors sensors (nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis (drivetrain, lateral_controller, angular_controller, sensors);

void initialize() {
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	arm.set_value(true);
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

		pros::delay(20);
	}
}