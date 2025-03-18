#include "chisel/util/motoritf.h"

#include <util.h>

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

void MotorItf::update() {
	auto top_command = command_queue.top();
	if (top_command->life <= 0) {
		command_queue.pop();
		printf("%sMotorItf command died, recursively finding next command\n", prefix().c_str());
		update();
		return;
	}

	if (top_command->power >= -127 && top_command->power <= 127) {
		motor->move(top_command->power);
	} else {
		bool success = motor->set_brake_mode(
			(top_command->power == MOTOR_HOLD) ? pros::E_MOTOR_BRAKE_HOLD :
			(top_command->power == MOTOR_BRAKE) ? pros::E_MOTOR_BRAKE_BRAKE :
			(top_command->power == MOTOR_COAST) ? pros::E_MOTOR_BRAKE_COAST :
			nullptr
		);

		if (!success) {
			printf("%sassumed motor stop, but value was out of range\n", prefix().c_str());
		}
	}
}

} // namespace chisel