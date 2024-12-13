#ifndef DRIVE_CURVE_H
#define DRIVE_CURVE_H

#include "util.h"

namespace chisel {
    class drive_curve {
        public:

        float deadband = 0;
        float min_output = 0;
        float expo_curve = 1;

        drive_curve(float deadband, float min_output, float expo_curve):
                deadband(deadband), min_output(min_output), expo_curve(expo_curve) {}

        float calculate(float input) {
            if (std::abs(input) <= deadband) return 0;

            int sign = sgn(input);

            float adj = 127/ std::pow(127, expo_curve);
            float expo = std::pow(std::abs(value), expo_curve);

            float output = (float) (adj * expo);

            return sign * std::max(min_output, output);
        }
    };
}

#endif //DRIVE_CURVE_H