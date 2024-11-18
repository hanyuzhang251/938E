#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <cstddef>

// constants

const double EXP = std::pow(128, 1/127);
const double RADIAN = M_PI / 180;

constexpr int TILE_SIDE = 24;

constexpr int WHEEL_TICKS_PER_ROTATION = 900;

constexpr float WHEEL_DIAMETER = 2.75;
const float WHEEL_CIRCUM = M_PI * WHEEL_DIAMETER;

const int TICKS_PER_TILE = TILE_SIDE / WHEEL_CIRCUM * WHEEL_TICKS_PER_ROTATION;

constexpr int INTAKE_SPEED = 100;

// config

constexpr int DRIVE_TRAIN_LEFT_FRONT_MOTOR_PORT = -15;
constexpr int DIRVE_TRAIN_LEFT_MIDDLE_MOTOR_PORT = -1;
constexpr int DRIVE_TRAIN_LEFT_BACK_MOTOR_PORT = -20;

constexpr int DRIVE_TRAIN_RIGHT_FRONT_MOTOR_PORT = 14;
constexpr int DRIVE_TRAIN_RIGHT_MIDDLE_MOTOR_PORT = 18;
constexpr int DRIVE_TRAIN_RIGHT_BACK_MOTOR_PORT = 16;

constexpr int INTAKE_BOTTOM_PORT = 5;
constexpr int INTAKE_TOP_PORT = -2;

constexpr int MOGO_CLAMP_PORT = 1;
constexpr int MOGO_BAR_PORT = 3;

constexpr int ARM_PORT = 4;
constexpr int ARM_END_PORT = 8;

constexpr int IMU_PORT = 21;

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

pros::MotorGroup intake_both ({
	INTAKE_BOTTOM_PORT,
	INTAKE_TOP_PORT
});

pros::adi::DigitalOut mogo_clamp_piston (MOGO_CLAMP_PORT);
pros::adi::DigitalOut mogo_bar_piston (MOGO_BAR_PORT);

pros::adi::DigitalOut arm (ARM_PORT);
pros::adi::DigitalOut arm_end (ARM_END_PORT);

pros::IMU imu (IMU_PORT);


lemlib::Drivetrain drivetrain(&drive_train_left_motor_group, &drive_train_right_motor_group, 11, lemlib::Omniwheel::NEW_275, 600, 2);

lemlib::OdomSensors sensors (nullptr, nullptr, nullptr, nullptr, &imu);


lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              0.2, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              0.2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              1, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis (drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);

void initialize() {
	arm.set_value(true);

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

void competition_initialize() {	
	arm.set_value(true);
}

ASSET(test_txt)
ASSET(rightGrabRings_txt)

void autonomous() {
	chassis.setPose(61.256, 9.955, 90);

	// arm.set_value(false);
	// pros::delay(250);
	// arm_end.set_value(true);
	// pros::delay(250);
	// arm.set_value(true);
	// pros::delay(250);

	chassis.moveToPoint(49.045, 9.955, 1000, {.forwards=false});

	chassis.waitUntilDone();
	chassis.turnToPoint(70, -1, 1000);

	chassis.waitUntilDone();
	chassis.moveToPoint(58.47, 5.4, 1000);

	chassis.waitUntilDone();
	chassis.moveToPose(27.16, 21.727, 115, 1000, {.forwards = false});

	chassis.waitUntil(25);
	mogo_clamp_piston.set_value(true);

	chassis.waitUntilDone();
	chassis.moveToPose(36.609, 46.67, 268.5, 5000);

	// chassis.follow(test_txt, 10, 10000, true, false);
	
	// chassis.waitUntil(34);
	// mogo_clamp_piston.set_value(true);
	// pros::delay(250);

	// chassis.turnToPoint(23.5, 47, 5000, {}, false);
	// intake_both.move(100);
	// chassis.moveToPoint(26.5, 40.5, 5000, {}, false);

	// chassis.turnToPoint(5, 32.5, 5000, {}, false);
	// intake_both.brake();

	// chassis.follow(rightGrabMogo_txt, 10, 5000);

	// chassis.waitUntil(12);
	// intake_both.move(100);

	// chassis.waitUntilDone();
	// intake_both.brake();
}

void opcontrol() {

    while (true) {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) intake_both.move(INTAKE_SPEED);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) intake_both.move(-INTAKE_SPEED);
		else intake_both.brake();
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) mogo_clamp_piston.set_value(true);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) mogo_clamp_piston.set_value(false);
		
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) arm.set_value(false);
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) arm.set_value(true);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			arm_end.set_value(true);
			arm.set_value(true);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		arm.set_value(false);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
		arm_end.set_value(true);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
		arm_end.set_value(false);
		}

        pros::delay(25);
    }
}