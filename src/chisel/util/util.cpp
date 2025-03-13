#include "util/util.h"
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

float deg_err(const float current, const float target) {
    float diff = std::fmod(current - target + 180.0f, 360.0f);
    
    if (diff < 0.0f) diff += 360.0f;

    return diff - 180.0f;
}

std::string format_millis(const uint32_t milliseconds) {
    uint32_t minutes = (milliseconds / 60000) % 60;
    uint32_t seconds = (milliseconds / 1000) % 60;
    uint32_t millis = milliseconds % 1000;
    
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << minutes << ":"
        << std::setw(2) << seconds << "."
        << std::setw(3) << millis;
    
    return oss.str();
}

std::string prefix() {
    char buff[15];
    std::snprintf(buff, 15, "[%s]:   ", format_millis(pros::millis()).c_str());

    return buff;
}

Toggle::Toggle(const bool value_) : value(value_), ptrigger(false) {}

void Toggle::tick(const bool trigger) {
    if (trigger && !ptrigger) value = !value;
    ptrigger = trigger;
}

} // namespace chisel