#include "main.h"
#include "chisel/pid_controller.h"

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

constexpr pros::anlg DRIVE_JOYSTICK = pros::CTRL_ANLG_LY;
constexpr pros::anlg TURN_JOYSTICK = pros::CTRL_ANLG_RX;

constexpr pros::digi INTAKE_FWD_BUTTON = pros::CTRL_DIGI_L1;
constexpr pros::digi INTAKE_REV_BUTTON = pros::CTRL_DIGI_L2;

constexpr pros::digi MOGO_ON_BUTTON = pros::CTRL_DIGI_A;
constexpr pros::digi MOGO_OFF_BUTTON = pros::CTRL_DIGI_B;

constexpr pros::digi ARM_UP_BUTTON = pros::CTRL_DIGI_R1;
constexpr pros::digi ARM_DOWN_BUTTON = pros::CTRL_DIGI_R2;

constexpr pros::digi ARM_END_ON_BUTTON = pros::CTRL_DIGI_RIGHT;
constexpr pros::digi ARM_END_OFF_BUTTON = pros::CTRL_DIGI_LEFT;

constexpr pros::digi RESET_ARM_POS_BUTTON = pros::CTRL_DIGI_X;
constexpr pros::digi FORCE_ARM_POS_BUTTON = pros::CTRL_DIGI_Y;

// PID CONTROLLERS

constexpr float LATERAL_PID_KP = 0.08;
constexpr float LATERAL_PID_KI = 0.003;
constexpr float LATERAL_PID_KD = 0.035;
constexpr float LATERAL_PID_WIND = 300;
constexpr float LATERAL_PID_SLEW = 999;

constexpr float ANGULAR_PID_KP = 0.8;
constexpr float ANGULAR_PID_KI = 0.1;
constexpr float ANGULAR_PID_KD = 1;
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
constexpr float MAX_ARM_HEIGHT = 2600;

constexpr float DRIVE_CURVE_DEADBAND = 3;
constexpr float DRIVE_CURVE_MIN_OUT = 10;
constexpr float DRIVE_CURVE_EXPO_CURVE = 3;

constexpr float TURN_CURVE_DEADBAND = 3;
constexpr float TURN_CURVE_MIN_OUT = 10;
constexpr float TURN_CURVE_EXPO_CURVE = 3;



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