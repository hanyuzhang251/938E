#pragma once

#include "util.h"
#include <functional>

namespace chisel {

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