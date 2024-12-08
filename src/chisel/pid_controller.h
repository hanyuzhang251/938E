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
        float slew = 999;

        std::atomic<float>& value;
        std::atomic<float>& target;
        std::atomic<float>& output;

        std::function<float(float, float)> normalize_func = nullptr;

        pid_controller(
                float kp, float ki, float kd, float i_clamp, float i_range, float slew,
                std::atomic<float>& value, std::atomic<float>& target, std::atomic<float>& output):
                        kp(kp), ki(ki), kd(kd), i_clamp(i_clamp), i_range(i_range), slew(slew),
                        value(value), target(target), output(output) {};

        float calculate(float error, float integral, float derivative) {
            return error * kp + integral * ki + derivative * kd;
        }

        float prev_output = 0;
        float prev_error = 0;

        float error = 0;
        float integral = 0;
        float derivative = 0;

        void update() {
            prev_output = output.load();
            prev_error = error;

            error = target.load() - value.load();
            if (normalize_func) error = normalize_func(target.load(), value.load());

            if (sgn(prev_error) != sgn(error)) integral = 0;
            if (std::abs(error) <= i_range) integral += error;
            else integral = 0;
            std::clamp(integral, -i_clamp, i_clamp);

            derivative = error - prev_error;

            float calc_power = calculate(error, integral, derivative);
            calc_power = std::max(prev_output - slew, std::min(prev_output + slew, calc_power));

            output.store(calc_power);
        }
    };
}

#endif //PID_CONTROLLER_H