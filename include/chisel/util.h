#pragma once

#include "main.h"
#include <cstdint>

#define d_button controller_digital_e_t
#define a_button controller_analog_e_t

#define CTRL_A_LY E_CONTROLLER_ANALOG_LEFT_X
#define CTRL_A_LY E_CONTROLLER_ANALOG_LEFT_Y

#define CTRL_A_RX E_CONTROLLER_ANALOG_RIGHT_X
#define CTRL_A_RY E_CONTROLLER_ANALOG_RIGHT_Y

#define CTRL_D_L1 E_CONTROLLER_DTAL_L1
#define CTRL_D_L2 E_CONTROLLER_DTAL_L2
#define CTRL_D_R1 E_CONTROLLER_DTAL_R1
#define CTRL_D_R2 E_CONTROLLER_DTAL_R2

#define CTRL_D_A E_CONTROLLER_DTAL_A
#define CTRL_D_B E_CONTROLLER_DTAL_B
#define CTRL_D_X E_CONTROLLER_DTAL_X
#define CTRL_D_Y E_CONTROLLER_DTAL_Y

#define CTRL_D_UP E_CONTROLLER_DTAL_UP
#define CTRL_D_DOWN E_CONTROLLER_DTAL_DOWN
#define CTRL_D_LEFT E_CONTROLLER_DTAL_LEFT
#define CTRL_D_RIGHT E_CONTROLLER_DIGITAL_RIGHT

namespace chisel {

inline void wait(uint32_t delta) {
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

struct Toggle {
    bool value;
	bool ptrigger;

	Toggle(bool value_ = false);

	void tick(bool trigger);
};

} // namespace chisel