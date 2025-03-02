#pragma once

#include "main.h"
#include <cstdint>

namespace chisel {

inline void wait(uint32_t delta) {
    pros::delay(delta);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
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