#pragma once

#include "util.h"
#include <atomic>
#include <functional>

namespace chisel {

struct PIDController {                                                         
    float kp;
    float ki;
    float kd;
    float wind; // range of error for integral accumulation
    float clamp; // clamps the integral
    float slew; // maximum acceleration and deceleration
    float small_error; // error range describing almost to target
    float large_error; // error range to start settling
    float tolerance; // acceptable error

    PIDController(
        float kp, float ki, float kd, float wind, float clamp, float slew,
        float small_error, float large_error, float tolerance
    );
};

struct PIDProcess {                                                            
    std::atomic<float>& value;
    std::atomic<float>& target;
    std::atomic<float>& output;
    const PIDController& pid;
    float max_speed;
    float min_speed;
    uint32_t life;
    std::function<float(float, float)> normalize_err;

    float prev_output = 0;
    float prev_error = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;

    PIDProcess(
        std::atomic<float>& value,
        std::atomic<float>& target,
        std::atomic<float>& output,
        const PIDController& pid,
        float max_speed,
        float min_speed,
        uint32_t life,
        std::function<float(float, float)> normalize_err =
            [](float err, float maxErr) { return err / maxErr; });

    float get_error();

    auto operator()();
};

void pid_handle_process(PIDProcess& process);

} // namespace chisel