#include "drive.h"

#include <utility>

namespace chisel {

DriveTrain::DriveTrain(
    pros::MotorGroup *left_motors, pros::MotorGroup *right_motors,
    float wheel_size, float track_width, float rpm
):  
    left_motors(left_motors), right_motors(right_motors),
    wheel_size(wheel_size), track_width(track_width), rpm(rpm) {
        printf("%screate new DriveTrain: ", prefix().c_str());

        printf("ld(");
        for (uint8_t port : left_motors->get_port_all()){
            printf("%d, ", port);
        }
        printf("\b\b) ");

        printf("rd(");
        for (uint8_t port : right_motors->get_port_all()){
            printf("%d, ", port);
        }
        printf("\b\b) ");

        printf("track_width=%f, wheel_size=%f, rpm=%f\n", track_width, wheel_size, rpm);
    };

DriveCurve::DriveCurve(const float deadband, const float min_out, std::function<float(float)> curve)
    : deadband(deadband), min_out(min_out), curve(std::move(curve)) {
        printf("%screate new DriveCurve: deadband=%f, min_out=%f\n", prefix().c_str(), deadband, min_out);
    }

float drive_calc_power(const float input, const DriveCurve& curve) {
    if (input < curve.deadband) return 0;

    float output = curve.curve(input);

    if (std::abs(curve.min_out) > std::abs(output)) {
        return sgn(output) * curve.min_out;
    }

    return output;
}

} // namespace chisel