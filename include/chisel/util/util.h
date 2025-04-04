#pragma once

#include "main.h"
#include "chisel/data/pose.h"
#include <iomanip>
#include <sstream>
#include <cstdint>

#define digi_button controller_digital_e_t
#define anlg_button controller_analog_e_t

#define CTRL_ANGL_LX E_CONTROLLER_ANALOG_LEFT_X
#define CTRL_ANGL_LY E_CONTROLLER_ANALOG_LEFT_Y

#define CTRL_ANGL_RX E_CONTROLLER_ANALOG_RIGHT_X
#define CTRL_ANGL_RY E_CONTROLLER_ANALOG_RIGHT_Y

#define CTRL_DIGI_L1 E_CONTROLLER_DTAL_L1
#define CTRL_DIGI_L2 E_CONTROLLER_DTAL_L2
#define CTRL_DIGI_R1 E_CONTROLLER_DTAL_R1
#define CTRL_DIGI_R2 E_CONTROLLER_DTAL_R2

#define CTRL_DIGI_A E_CONTROLLER_DTAL_A
#define CTRL_DIGI_B E_CONTROLLER_DTAL_B
#define CTRL_DIGI_X E_CONTROLLER_DTAL_X
#define CTRL_DIGI_Y E_CONTROLLER_DTAL_Y

#define CTRL_DIGI_UP E_CONTROLLER_DTAL_UP
#define CTRL_DIGI_DOWN E_CONTROLLER_DTAL_DOWN
#define CTRL_DIGI_LEFT E_CONTROLLER_DTAL_LEFT
#define CTRL_DIGI_RIGHT E_CONTROLLER_DIGITAL_RIGHT

constexpr int INIT_STATE = 0;
constexpr int AUTON_STATE = 1;
constexpr int DRIVE_STATE = 2;

namespace chisel {

inline void wait(const uint32_t delta) {
    pros::delay(delta);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T> T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float deg_norm(float degree);

float deg_err(float current, float target);

float deg_to_point(const Pose& point);

float dist_to_point(Pose point);

std::string format_millis(uint32_t milliseconds);

std::string prefix();

struct Toggle {
    bool value;
	bool ptrigger;

	explicit Toggle(bool value_ = false);

	void tick(bool trigger);
};

} // namespace chisel