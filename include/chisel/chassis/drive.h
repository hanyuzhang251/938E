#pragma once

#include "main.h"
#include "chisel/util/util.h"
#include <functional>
#include <cmath>

namespace chisel {

struct DriveTrain {
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;

    float wheel_size;
    float track_width;
    float gear_ratio;

    DriveTrain(
        pros::MotorGroup *left_motors, pros::MotorGroup *right_motors,
        float wheel_size, float track_width, float gear_ratio
    );
};

constexpr auto LINEAR_CURVE = [](const float input) {return input;};
constexpr auto SIGMOID_CURVE = [](const float input) {
    // finer control at high speeds
    return 127 * std::sin((M_PI * input) / 254);
};
constexpr auto SIGMA_CURVE = [](const float input) {
    // finer control at low speeds
    return 254 * 1 / M_PI / std::asin(input / 127);
};

struct DriveSettings {
    int32_t deadband;
    int32_t min_out;
    std::function<int32_t(int32_t)> curve;

    DriveSettings(
        int32_t deadband,
        int32_t min_out,
        std::function<int32_t(int32_t)> curve = [](const int32_t input) {return input;});

    int32_t drive_calc_power(int32_t input) const;
};

}