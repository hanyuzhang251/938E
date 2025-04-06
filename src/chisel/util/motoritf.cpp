#include "chisel/util/motoritf.h"

#include "chisel/util/util.h"

namespace chisel {

void Command::dismiss() {
	life = -1;
}

Command::Command(const int32_t power, const int32_t priority, const uint32_t life):
	power(power), priority(priority), life(life) {}

MotorItf::MotorItf(pros::Motor* motor): motor(motor) {}

void MotorItf::assign_command(Command* command) {
	command_list.push_back(command);
}

void MotorItf::clean_commands() {
	// If the command list is empty, return.
	// Caller of method should also check if the command queue is empty.
	if (command_list.empty()) return;

	erase_if(command_list, [](const Command* command) {return command->life <= 0;});
}

void MotorItf::update() {
	// If the command queue is empty, return.
	// Caller of method should also check if the command queue is empty.
	if (command_list.empty()) return;

	const Command* top_command = command_list[0];
	for (const auto command : command_list) {
		--command->life;

		if (command->priority > top_command->priority) {
			top_command = command;
		}
	}

	// If the power reported by the top command is valid, set the final power (to be sent to the motor).
	if (abs(top_command->power) <= 127) {
	    final_power = top_command->power;
	}
	// Otherwise, ignore it and report to the console.
	// Perhaps it would be better to set it anyway as it's more likely a motor power of >127.0f was intended to be 127.0f.
	// Regardless, this system neither has caused any problems nor likely will cause any problems, so I'll leave it like this.
	else {
		printf("%sinvalid motor power (%ld) sent to motorItf, ignoring command\n", prefix().c_str(), top_command->power);
	}
}

void MotorItf::push_control() const {
	(void)motor->move(final_power);
}

} // namespace chisel