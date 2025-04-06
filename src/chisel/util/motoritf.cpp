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
	int clear_count = 0;
	while (true) {
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
	const auto top_command = command_queue.top();
	--top_command->life;

	if (top_command->power >= -127 && top_command->power <= 127) {
		final_power = top_command->power;
	} else {
		final_power = 0;
		final_motor_brake_mode =
			(top_command->power == MOTOR_HOLD) ? pros::E_MOTOR_BRAKE_HOLD :
				(top_command->power == MOTOR_BRAKE) ? pros::E_MOTOR_BRAKE_BRAKE :
					(top_command->power == MOTOR_COAST) ? pros::E_MOTOR_BRAKE_COAST :
					pros::E_MOTOR_BRAKE_INVALID;
	}
}

bool MotorItf::push_update() const {
	bool success = motor->set_brake_mode(final_motor_brake_mode);
	success = success && motor->move(final_power);
	return success;
}

} // namespace chisel