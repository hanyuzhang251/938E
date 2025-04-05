#pragma once

#include "util/util.h"

#include <atomic>
#include <functional>

namespace chisel {

/**
 * @brief Stores settings used in PID controllers
 */
struct PIDSetting {
    float kp;
    float ki;
    float kd;
    float tolerance;
    float wind;
    float clamp;
    float slew;
    float small_error;
    float large_error;

    /**
     * @brief PIDSetting constructor
     *
     * @param kp Proportional component, provides the majority of power for the system.
     * @param ki Integral component, corrects for steady state error. Shouldn't be very high.
     * @param kd Derivative component, dampens the proportional component.
     * \n
     * @param tolerance Range of error considered acceptable
     * \n
     * @param wind Range of error to start accumulating integral.
     *        if set to MAXFLOAT (default), integral will always accumulate
     * @param clamp Absolute max of integral.
     *        If set to MAXFLOAT (default), integral will not be clamped.
     * @param slew Absolute max rate of change of output.
     *        If set to MAXFLOAT (default), output acceleration will not be affected.
     * \n
     * @param small_error While error approaches zero in this range, integral will lose influence to mitigate overshooting.
     *        If set to 0.0f (default), integral influence will not be affected.
     * @param large_error While absolute error approaches zero in this range, derivative will gain influence to increase the speed of long movements.
     *        Outside of this range derivative does not have any impact.
     *        If set to MAXFLOAT (default), derivative influence will be present regardless of error.
     */
    PIDSetting(
        float kp, float ki, float kd, float tolerance, float wind = MAXFLOAT, float clamp = MAXFLOAT, float slew = MAXFLOAT,
        float small_error = 0, float large_error = MAXFLOAT
    );
};

/**
 * @brief PID controller, calculates an output based on the current value and target.
 *
 * Stores the PID settings and the current state of the controller.
 * Provided atomic<float> references are where the controller gets the current value and target, as well as storing the output.
 * Additionally maintains min and max output variables, life of the controller, and an optional custom error calculation.
 */
struct PIDController {
    std::atomic<float>& value;
    std::atomic<float>& target;
    std::atomic<float>& output;
    const PIDSetting& pid;
    float min_speed;
    float max_speed;
    uint32_t life;
    std::function<float(float, float)> normalize_err;

    float prev_output = 0;
    float prev_error = 0;
    float error = 0;
    float integral = 0;
    float derivative = 0;

    /**
     * @brief PIDController constructor
     *
     * @param value atomic<float> reference, where the controller gets the current value.
     * @param target atomic<float> reference, where the controller gets the target value.
     * @param output atomic<float> reference, where the controller stores the output.
     * \n
     * @param pid Reference to settings used by the controller. The controller will not modify these settings.
     * @param max_speed Absolute max value of the output.
     * @param min_speed Absolute min value of the output. Currently not in use.
     * @param life Life of the controller in ticks.
     * @param normalize_err Custom calculation for error provided the target and current values.
     *        If set to nullptr (default), error will be calculated through [target_value] - [current value].
     *        The parameter name does not very well describe its purpose.
     */
    PIDController(
        std::atomic<float>& value,
        std::atomic<float>& target,
        std::atomic<float>& output,
        const PIDSetting& pid,
        float min_speed,
        float max_speed,
        uint32_t life,
        const std::function<float(float, float)>& normalize_err = nullptr);

    /**
     * @return The error based on the current state of the controller.
     *
     * @note Calculats the error based on the state of the controller,
     *       which depending on when the controller was last updated may not match the controller's error value.
     *       When wanting to get the controller's error value, directly access the variable [error]
     */
    [[nodiscard]] float get_error() const;

    /**
     * @return A structural binding of all variables.
     */
    auto operator()();
};

/**
 * @brief Updates a provided PID controller.
 *
 * @param process Reference to the PID controller to update.
 */
void pid_handle_process(PIDController& process);

} // namespace chisel