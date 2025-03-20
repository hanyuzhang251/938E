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
    float deadband;
    float min_out;
    std::function<float(float)> curve;

    DriveSettings(
        float deadband,
        float min_out,
        std::function<float(float)> curve = [](const float input) {return input;});
};

float drive_calc_power(float input, DriveSettings& curve);

}