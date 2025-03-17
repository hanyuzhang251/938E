#include "chisel/util/motoritf.h"

namespace chisel {

void Command::dismiss() {
	life = -1;
}

Command::Command(float power, int priority, uint32_t life):
	power(power), priority(priority), life(life) {}

MotorItf::MotorItf(pros::Motor* motor): motor(motor) {}

void MotorItf::assign_command(Command* command) {
	command_queue.emplace(command);
}
	
} // namespace chisel