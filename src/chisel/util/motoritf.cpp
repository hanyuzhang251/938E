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
	if (command_queue.empty()) return 0;

	int clear_count = 0;

	while (!command_queue.empty()) {
		if (const auto top_comand = command_queue.top(); top_comand->life <= 0) {
			command_queue.pop();
			++clear_count;
		} else {
			break;
		}
	}
	return clear_count;
}


void MotorItf::update() {
	if (command_queue.empty()) return;

	const auto top_command = command_queue.top();
	--top_command->life;

	if (abs(top_command->power) <= 127) {
		final_power = top_command->power;
	} else {
		printf("%sinvalid motor power (%ld) sent to motorItf, ignoring command\n", prefix().c_str(), top_command->power);
	}
}

void MotorItf::push_update() const {
	(void)motor->move(final_power);
}

} // namespace chisel