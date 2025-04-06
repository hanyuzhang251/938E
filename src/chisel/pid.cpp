#include "pid.h"

namespace chisel {

PIDSetting::PIDSetting(
    const float kp,
    const float ki,
    const float kd,
    const float tolerance,
    const float wind,
    const float clamp,
    const float slew,
    const float small_error,
    const float large_error
):  kp(kp),
    ki(ki),
    kd(kd),
    tolerance(tolerance),
    wind(wind),
    clamp(clamp),
    slew(slew),
    small_error(small_error),
    large_error(large_error)
{
        printf("%screate new PIDController: kp=%f, ki=%f, kd=%f, tolerance=%f, wind=%f, clamp=%f, slew=%f, small_err=%f, large_err=%f\n",
            prefix().c_str(), kp, ki, kd, tolerance, wind, clamp, slew, small_error, large_error);
}

PIDController::PIDController(
    std::atomic<float>& value,
    std::atomic<float>& target,
    std::atomic<float>& output,
    const PIDSetting& pid,
    const float min_speed, const float max_speed,
    const uint32_t life,
    const std::function<float(float, float)>& normalize_err
):  value(value),
    target(target),
    output(output),
    pid(pid),
    min_speed(min_speed),
    max_speed(max_speed),
    life(life),
    normalize_err(normalize_err)
{
        printf("%screate new PIDProcess: max_speed=%f, min_speed=%f, life=%d, %s\n", prefix().c_str(), max_speed, min_speed, life, (!normalize_err ? "default err calc" : "custom err calc"));
}

float PIDController::get_error() const {
    // If a custom error calculation is provided, use it.
    if (normalize_err) {
        return normalize_err(target.load(), value.load());
    }
    // Otherwise, use the default calculation of target - value.
    else {
        return target.load() - value.load();
    }
}

auto PIDController::operator()() {
    return std::tie(
        value, target, output, pid, min_speed, max_speed, life,
        normalize_err, prev_output, prev_error, error, integral,
        derivative);
}

void pid_handle_process(PIDController& process) {
    printf("\t\tPID PROCESS UPDATE\n");
    // Structural binding of PID controller for conciseness.
    auto [value, target, output, pid, min_speed, max_speed, life, normalize_err,
          prev_output, prev_error, error, integral, derivative] = process();

    // If the controller is no longer alive, don't update it.
    // In addition, caller of the function should also ensure the PID controller is still alive.
    if (life <= 0) {
        printf("\t\tpid process died, returning\n");
        return;
    }
    life -= 1;
    printf("\t\tpid process life=%ld\n", life);

    // Update prev values.
    prev_output = output.load();
    prev_error = error;
    printf("\t\tp-out=%fd, p-err=%d\n", prev_output, prev_error);

    // Update the current error.
    // While the get_error() method does not normally ensure continuity with the current state,
    // it's okay here because we're updating the controller right now.
    error = process.get_error();
    printf("\t\terror=%f", error);

    // If we crossed the target, we should set the integral to zero.
    if (sgn(prev_error) != sgn(error)) {
        printf("\t\tintegral crossed, setting to 0\n");
		integral = 0;
	}

    // If the integral is in the windup range, update integral.
    if (std::abs(error) <= pid.wind) {
        integral += error;
        printf("\t\terror in range (%f), integral=%f\n", pid.wind, integral);
    }
    // Otherwise, set integral to zero.
    else {
        printf("\t\terror is not in range (%f), integral=0\n", pid.wind);
        integral = 0;
    }

    // Clamp the integral.
    integral = clamp(integral, -pid.clamp, pid.clamp);
    printf("\t\tclamp integral to range (%f), integral=%f\n", pid.clamp, integral);
    
    // Update derivative
    derivative = error - prev_error;
    printf("\t\tderivative=%f\n", derivative);

    // Not sure why we create a const instance of the error,
    // but this was written two bots ago and it worked then so I'm gonna keep it like this.
    const float real_error = error;

    // Copy integral and derivative to avoid messing with originals.
    float real_integral = integral;
    float real_derivative = derivative;

    // Integral *= [error=0.0f -> error=small_error : 0.3f -> 1.0f]
    real_integral *= std::min(1.0f, std::max(0.3f, std::abs(error) / pid.small_error));
    // Derivative *= [error=0.0f -> error=large_error : 1.0f -> 0.0f]
	real_derivative *= 1 - std::min(1.0f, std::abs(error) / pid.large_error);

    // Calculate power.
    float calc_power = real_error * pid.kp + real_integral * pid.ki + real_derivative * pid.kd;
    printf("\t\traw output=%f\n", calc_power);

    // Clamp power to slew.
    calc_power = clamp(calc_power, prev_output - pid.slew, prev_output + pid.slew);
    printf("\t\tclamp output to range (%f), output=%f\n", pid.slew, calc_power);
	// Clamp power to min/max speed
    calc_power = abs_clamp(calc_power, min_speed, max_speed);
    printf("\t\tclamp output to abs range (%f->%f), output=%f\n", min_speed, max_speed, calc_power);

    // Set output power.
    output.store(calc_power);
}

} // namespace chisel