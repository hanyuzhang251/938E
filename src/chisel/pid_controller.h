#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "util.h"

#include <algorithm>
#include <functional>
#include <atomic>

namespace chisel {
    class pid_controller {
        public:

        float kp = 0;
        float ki = 0;
        float kd = 0;
        float i_clamp = 999;
        float i_range = 999;
        float i_decay = 0;
        float slew = 999;

        at_f& value;
        at_f& target;

        std::function<float(float, float)> normalize_func = nullptr;

        pid_controller(float kp, float ki, float kd, float i_clamp, float i_range, float i_decay, float slew, at_f& value, at_f& target, at_f& output):
                kp(kp), ki(ki), kd(kd), i_clamp(i_clamp), i_range(i_range), i_decay(i_decay), slew(slew), value(value), target(target) {};

        float calculate(float error, float integral, float derivative) {
            return error * kp + integral * ki + derivative * kd;
        }

        float prev_output = 0;
        float prev_error = 0;

        float error = 0;
        float integral = 0;
        float derivative = 0;

        float update() {
            error = target.load() - value.load();
            if (normalize_func) error = normalize_func(target.load(), value.load());

            if (sgn(prev_error) != sgn(error)) integral = 0;
            if (std::abs(error) <= i_range) integral += error;
            else integral *= i_decay;
            std::clamp(integral, -i_clamp, i_clamp);

            derivative = error - prev_error;

            float calc_power = calculate(error, integral, derivative);
            calc_power = std::max(prev_output - slew, std::min(prev_output + slew, calc_power));

            prev_output = calc_power;
            prev_error = error;

            return calc_power;
        }
    };
}

#endif //PID_CONTROLLER_H