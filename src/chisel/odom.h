#ifndef ODOM_H
#define ODOM_H

#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "util.h"

namespace chisel {
    class odom {
        public:

        float track_width;

        float tpi;
        float threshold;

        pros::IMU imu;
        pros::Motor left_motor;
        pros::Motor right_motor;

        at_f& x_pos;
        at_f& y_pos;
        at_f& heading;

        odom(float track_width, float tpi, float threshold, pros::IMU imu, pros::Motor left_motor, pros::Motor right_motor, at_f& x_pos, at_f& y_pos, at_f& heading):
                track_width(track_width), tpi(tpi), threshold(threshold), imu(imu), left_motor(left_motor), right_motor(right_motor), x_pos(x_pos), y_pos(y_pos), heading(heading) {};

        float prev_left_pos = 0;
        float prev_right_pos = 0;

        pose calc_w_motor() {
            float left_dif = left_motor.get_position() - prev_left_pos;
            float right_dif = right_motor.get_position() - prev_left_pos;

            if (std::abs(left_dif - right_dif) <= threshold) {
                float d = (left_dif + right_dif) / 2;
                return pose({
                        x_pos + d * std::cos(heading.load()),
                        y_pos + d * std::sin(heading.load()),
                        heading.load()
                });
            }

            float arc_rad = track_width / 2 * (left_dif + right_dif) / (right_dif - left_dif);
            float ang_dif = (right_dif - left_dif) / track_width;
            
            float arc_cx = x_pos.load() - arc_rad * std::sin(heading.load());
            float arc_cy = y_pos.load() + arc_rad * std::cos(heading.load());

            return pose({
                    arc_cx + arc_rad * std::sin(heading.load() + ang_dif),
                    arc_cy + arc_rad * std::cos(heading.load() + ang_dif),
                    heading.load() + ang_dif
            });
        }

        pose calc_w_imu() {

        }

        void update() {
            pose motor_result = calc_w_motor();

        }
    };
}

#endif //ODOM_H