#include "pid.h"

namespace chisel {

PIDSettings::PIDSettings(
    const float kp, const float ki, const float kd, const float wind, const float clamp, const float slew,
    const float small_error, const float large_error, const float tolerance)
    : kp(kp), ki(ki), kd(kd), wind(wind), clamp(clamp), slew(slew),
    small_error(small_error), large_error(large_error), tolerance(tolerance) {
        printf("%screate new PIDController: kp=%f, ki=%f, kd=%f, wind=%f, clamp=%f, slew=%f, small_err=%f, large_err=%f, tolerance=%f\n",
            prefix().c_str(), kp, ki, kd, wind, clamp, slew, small_error, large_error, tolerance);
    }

PIDController::PIDController(
    std::atomic<float>& value,
    std::atomic<float>& target,
    std::atomic<float>& output,
    const PIDSettings& pid,
    const float max_speed, const float min_speed,
    const uint32_t life,
    const std::function<float(float, float)>& normalize_err)
    : value(value),
      target(target),
      output(output),
      pid(pid),
      max_speed(max_speed), min_speed(min_speed),
      life(life),
      normalize_err(normalize_err) {
        printf("%screate new PIDProcess: max_speed=%f, min_speed=%f, life=%d, %s\n", prefix().c_str(), max_speed, min_speed, life, (!normalize_err ? "default err calc" : "custom err calc"));
      }

float PIDController::get_error() const {
    if (normalize_err) {
        return normalize_err(target.load(), value.load());
    } else {
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

    auto [value, target, output, pid, min_speed, max_speed, life, normalize_err,
          prev_output, prev_error, error, integral, derivative] = process();

    if (life <= 0) return;
    life -= 1;

    prev_output = output.load();
    prev_error = error;

    // error update
    error = process.get_error();

    // reset integral if we crossed target                                     
    if (sgn(prev_error) != sgn(error)) {
		integral = 0;
	}

    // update integral if error in windup range
    if (std::abs(error) <= pid.wind) {
		integral += error;
	}
    // set integral to zero
    else {
		integral = 0;
	}

    // clamp integral
    integral = clamp(integral, -pid.clamp, pid.clamp);
    
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
    calc_power = clamp(calc_power, prev_output - pid.slew, prev_output + pid.slew);
	// constrain power to min/max speed
    calc_power = clamp(calc_power, -max_speed, max_speed);

    // set output power
    output.store(calc_power);
}

} // namespace chisel