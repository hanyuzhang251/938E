#include "main.h"
#include "pros/misc.h"

#include <cmath>
#include <atomic>

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

struct pose {
    float x;
    float y;
    float h;
};

struct pid_controller {
    float kp;
    float ki;
    float kd;
    float wind;
    float slew;
};

struct drive_curve {
    float deadband;
    float min_out;
    float expo_curve;
};



/*****************************************************************************/
/*                                  CONFIG                                   */
/*****************************************************************************/

constexpr long PROCESS_DELAY = 15;
constexpr long LONG_DELAY = 15;

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

// BUTTONS

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

// PID CONTROLLERS

constexpr pid_controller lateral_pid (0.045, 0.003, 0.035, 300, 999);
constexpr pid_controller angular_pid (0.8, 0.1, 1, 30, 999);
constexpr pid_controller arm_pid (0.6, 0.02, 0, 90, 999);

// AUTON

constexpr float DIST_MULTI = 35.5;

// DRIVING

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

constexpr drive_curve lateral (3, 10, 3);
constexpr drive_curve angular (3, 10, 3);



/*****************************************************************************/
/*                              PID CONTROLLER                               */
/*****************************************************************************/

float pid_calc_power(int error, int integral, int derivative, const pid_controller& pid) {
    return error * pid.kp + integral * pid.ki + derivative * pid.kd;
}