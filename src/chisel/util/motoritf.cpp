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
	command_queue.emplace(command);
}

int MotorItf::clean_commands() {
	// If the command queue is empty, return.
	// Caller of method should also check if the command queue is empty.
	if (command_queue.empty()) return 0;

	int clear_count = 0;

	while (!command_queue.empty()) {
		// If the top command's life is <= 0, remove it from the queue.
		if (const auto top_comand = command_queue.top(); top_comand->life <= 0) {
			command_queue.pop();
			++clear_count;
		}
		// Otherwise, we have finished cleaning the commands. Break out of while loop.
		else {
			break;
		}
	}

	// Return the number of commands cleaned.
	return clear_count;
}


void MotorItf::update() {
	// If the command queue is empty, return.
	// Caller of method should also check if the command queue is empty.
	if (command_queue.empty()) return;

	// Get the top command and decrement its life.
	const auto top_command = command_queue.top();
	--top_command->life;

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