#include "drive.h"

namespace chisel {

DriveCurve::DriveCurve(float deadband, float min_out, std::function<float(float)> curve)
    : deadband(deadband), min_out(min_out), curve(curve) {
        printf("%screate new DriveCurve: deadband(%f), min_out(%f)", prefix(), deadband, min_out);
    }

float drive_calc_power(float input, DriveCurve& curve) {
    if (input < curve.deadband) return 0;

    float output = curve.curve(input);

    if (std::abs(curve.min_out) > std::abs(output)) {
        return sgn(output) * curve.min_out;
    }

    return output;
}

} // namespace chisel