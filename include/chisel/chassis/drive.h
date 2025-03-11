#pragma once

#include "main.h"
#include "chisel/util.h"
#include <functional>
#include <cmath>

namespace chisel {

struct DriveTrain {
    pros::MotorGroup *left_motors;
    pros::MotorGroup *right_motors;

    float wheel_size;
    float track_width;
    float rpm;

    DriveTrain(
        pros::MotorGroup *left_motors, pros::MotorGroup *right_motors,
        float wheel_size, float track_width, float rpm
    );
};

constexpr auto linear_curve = [](float input) {return input;};
constexpr auto sigmoid_curve = [](float input) {
    // finer control at high speeds
    return 127 * std::sin((M_PI * input) / 254);
};
constexpr auto sigma_curve = [](float input) {
    // finer control at low speeds
    return 254 * 1 / M_PI / std::asin(input / 127);
};

struct DriveCurve {
    float deadband;
    float min_out;
    std::function<float(float)> curve;

    DriveCurve(
        float deadband,
        float min_out,
        std::function<float(float)> curve = [](float input) {return input;});
};

float drive_calc_power(float input, DriveCurve& curve);

}