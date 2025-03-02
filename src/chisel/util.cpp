#include "util.h"
#include <cmath>

namespace chisel {

float deg_norm(float degree) {
    degree = std::fmod(degree, 360.0f);

    if (degree >= 180.0f) {
        degree -= 360.0f;
    } else if (degree < -180.0f) {
        degree += 360.0f;
    }

    return degree;
}

float deg_err(float current, float target) {
    float diff = std::fmod(current - target + 180.0f, 360.0f);
    
    if (diff < 0.0f) diff += 360.0f;

    return diff - 180.0f;
}

Toggle::Toggle(bool value_) : value(value_), ptrigger(false) {}

void Toggle::tick(bool trigger) {
    if (trigger && !ptrigger) value = !value;
    ptrigger = trigger;
}

} // namespace chisel