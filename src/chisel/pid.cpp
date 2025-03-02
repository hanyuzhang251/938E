#include "pid.h"

namespace chisel {

PIDProcess::PIDProcess(
    std::atomic<float>& value,
    std::atomic<float>& target,
    std::atomic<float>& output,
    const PIDController& pid,
    float max_speed,
    float min_speed,
    uint32_t life,
    std::function<float(float, float)> normalize_err)
    : value(value),
      target(target),
      output(output),
      pid(pid),
      max_speed(max_speed),
      min_speed(min_speed),
      life(life),
      normalize_err(normalize_err) {}

float PIDProcess::get_error() {
    if (normalize_err) {
        return normalize_err(target.load(), value.load());
    } else {
        return target.load() - value.load();
    }
}

auto PIDProcess::operator()() {
    return std::tie(
        value, target, output, pid, min_speed, max_speed, life,
        normalize_err, prev_output, prev_error, error, integral,
        derivative);
}

void pid_handle_process(PIDProcess& process) {                                 

    auto [value, target, output, pid, min_speed, max_speed, life, normalize_err,
          prev_output, prev_error, error, integral, derivative] = process();

    if (life <= 0) return;
    life -= 1;

    prev_output = output.load();
    prev_error = error;

    // error update
    error = target.load() - value.load();
    // normalize error if applicable
    if (normalize_err) {
		error = normalize_err(target.load(), value.load());
	}

    // reset integral if we crossed target                                     
    if (sgn(prev_error) != sgn(error)) {
		integral = 0;
	}

    // update integral if error in windup range
    if (std::abs(error) <= pid.wind) {
		integral += error;
	}
    // else decay integral
    else {
		integral *= pid.decay;
	}

    // clamp integral
    integral = std::min(pid.clamp, std::max(-pid.clamp, integral));
    
    // derivative update                                                       
    derivative = error - prev_error;

    float real_error = error;                                                  
    float real_integral = integral;
    float real_derivative = derivative;

    // scale integral on small error
    real_integral *= (std::min(
		1.0f,
		std::max(0.3f, std::abs(error) / pid.small_error))
	);
    // scale derivative on large error
	real_derivative *= (1 - std::min(1.0f, std::abs(error) / pid.large_error));

    // calculate power
    float calc_power =
		real_error * pid.kp
		+ real_integral * pid.ki
		+ real_derivative * pid.kd;
    // constrain power to slew
    calc_power = std::min(
		prev_output + pid.slew,
		std::max(prev_output - pid.slew, calc_power)
	);
	// constrain power to min/max speed
    calc_power = std::min(max_speed, std::max(-max_speed, calc_power));

    // set output power
    output.store(calc_power);
}

}