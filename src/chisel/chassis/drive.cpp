#include "drive.h"

namespace chisel {

DriveTrain::DriveTrain(
    pros::MotorGroup *left_motors, pros::MotorGroup *right_motors,
    const float wheel_diameter, const float track_width, const float gear_ratio
):  
    left_motors(left_motors), right_motors(right_motors),
    wheel_size(wheel_diameter), track_width(track_width), gear_ratio(gear_ratio) {
        // Multiply by 1.2f here cause the tracking is off. Why? idfk
        magic_number = gear_ratio / 360 * M_PI * wheel_diameter * 1.2f;

        printf("%screate new DriveTrain: ", prefix().c_str());

        printf("ld(");
        for (const uint8_t port : left_motors->get_port_all()){
            printf("%d, ", port);
        }
        printf("\b\b) ");

        printf("rd(");
        for (const uint8_t port : right_motors->get_port_all()){
            printf("%d, ", port);
        }
        printf("\b\b) ");

        printf("track_width=%f, wheel_diameter=%f, gearing=%f, m-num=%f\n", track_width, wheel_diameter, gear_ratio, magic_number);
    };

DriveSettings::DriveSettings(const int32_t deadband, const int32_t min_out, std::function<int32_t(int32_t)> curve)
    : deadband(deadband), min_out(min_out), curve(std::move(curve)) {
        printf("%screate new DriveCurve: deadband=%f, min_out=%f\n", prefix().c_str(), deadband, min_out);
    }

int32_t DriveSettings::drive_calc_power(const int32_t input) const {
    if (abs(input) < deadband) return 0;

    const int32_t output = curve(input);

    if (min_out > std::abs(output)) {
        return sgn(output) * min_out;
    }

    return output;
}

} // namespace chisel