#include "main.h"

#include "chisel/chisel.h"

constexpr int32_t DT_FL_PORT = -6;
constexpr int32_t DT_ML_PORT = -7;
constexpr int32_t DT_BL_PORT = -8;

constexpr int32_t DT_FR_PORT = 3;
constexpr int32_t DT_MR_PORT = 4;
constexpr int32_t DT_BR_PORT = 5;

constexpr int32_t INTAKE_PORT = -2;
constexpr int32_t ARM_PORT = -9;
constexpr int32_t MOGO_PORT = 21;
constexpr int32_t DOINKER_PORT = 21;

constexpr int32_t IMU_PORT = 21;
constexpr int32_t OPTICAL_PORT = 21;

constexpr pros::digi_button MENU_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_Y;
chisel::Toggle menu_toggle(false);
constexpr pros::digi_button MENU_SELECT_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
chisel::Toggle menu_select_toggle(false);
constexpr pros::digi_button MENU_BACK_BUTTON = pros::E_CONTROLLER_DIGITAL_B;
chisel::Toggle menu_back_toggle(false);

constexpr pros::digi_button MENU_POINTER_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_DOWN;
chisel::Toggle menu_pointer_down_toggle(false);
constexpr pros::digi_button MENU_POINTER_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
chisel::Toggle menu_pointer_up_toggle(false);

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::anlg_button TURN_JOYSTICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;

constexpr pros::digi_button DOINKER_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
chisel::Toggle doinker_toggle(false);

constexpr pros::digi_button MOGO_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
chisel::Toggle mogo_toggle(false);

constexpr pros::digi_button ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;

constexpr pros::digi_button ARM_MACRO_CYCLE_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
chisel::Toggle arm_macro_cycle_toggle(false);

pros::MotorGroup left_motors({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
pros::MotorGroup right_motors({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

pros::Motor intake(INTAKE_PORT);
auto intake_itf = chisel::MotorItf(&intake);
chisel::Command driver_intake_command = {0, 1};

pros::Motor arm(ARM_PORT);
auto arm_itf = chisel::MotorItf(&arm);
chisel::Command driver_arm_command = {0, 1};

constexpr float ARM_SPEED = 220;

constexpr float MAX_ARM_POS = 2000;
constexpr float ARM_LOAD_POS = 250;
constexpr float ARM_SCORE_POS = 800;

int arm_macro_cycle_index = 0;

chisel::PIDSettings arm_pid_settings{
    0.6, 0.1, 3, 70, 999, 999, 0, 0, 0
};
std::atomic<float> arm_pos(0);
std::atomic<float> arm_target_pos(0);
std::atomic<float> arm_pid_output(0);
chisel::PIDController arm_pid = {
    arm_pos,
    arm_target_pos,
    arm_pid_output,
    arm_pid_settings,
    127,
    0,
    1000 * 60 * 20,
    nullptr
};

pros::adi::DigitalOut mogo(MOGO_PORT);
pros::adi::DigitalOut doinker(DOINKER_PORT);

pros::Imu imu(IMU_PORT);
pros::Optical optical(OPTICAL_PORT);

chisel::DriveTrain drive_train(
    &left_motors,
    &right_motors,
    2.75f, // wheel diameter
    15, // track width
    450 // wheel rpm
);

chisel::DriveSettings lateral_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};
chisel::DriveSettings angular_drive_settings = {
    3,
    10,
    chisel::LINEAR_CURVE
};

chisel::Odom odom{
    {0, 0, 0},
    {0, 0, 0},
    &imu,
    &drive_train,
    nullptr,
    0
};

chisel::PIDSettings angular_pid_settings{
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::PIDSettings lateral_pid_settings{
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::Chassis chassis = {
    &drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_settings, &lateral_pid_settings,
    true
};

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool alliance = true; // true = red, false = blue
bool color_sort_enabled = false;

bool block_movement = false;
bool block_controls = false;

char ctrl_log[3][15];
int menu_page = 0;
int pointer_index = 0;

uint32_t next_display_time = pros::millis();
int next_display_index = 0;

void menu_update() {
    menu_toggle.tick(master.get_digital(MENU_TOGGLE_BUTTON));
    if (menu_toggle.value) {
        menu_toggle.value = false;

        pointer_index = 0;
        if (menu_page == 0) {
            menu_page = 1;
        } else {
            menu_page = 0;
        }
    }

    menu_pointer_up_toggle.tick(master.get_digital(MENU_POINTER_UP_BUTTON));
    if (menu_pointer_up_toggle.value) {
        menu_pointer_up_toggle.value = false;
        if (pointer_index > 0) --pointer_index;
    }

    menu_pointer_down_toggle.tick(master.get_digital(MENU_POINTER_DOWN_BUTTON));
    if (menu_pointer_down_toggle.value) {
        menu_pointer_down_toggle.value = false;
        ++pointer_index;
    }

    bool menu_select = false;
    menu_select_toggle.tick(master.get_digital(MENU_SELECT_BUTTON));
    if (menu_select_toggle.value) {
        menu_select_toggle.value = false;
        menu_select = true;
        pointer_index = 0;
    }

    bool menu_back = false;
    menu_back_toggle.tick(master.get_digital(MENU_BACK_BUTTON));
    if (menu_back_toggle.value) {
        menu_back_toggle.value = false;
        menu_back = true;
        pointer_index = 0;
    }

    switch (menu_page) {
        case 0: {
            block_controls = false;
            std::snprintf(ctrl_log[0], 15, "ut: %s         ", chisel::prefix().substr(1, 5).c_str());
            std::snprintf(ctrl_log[1], 15, "               ");
            std::snprintf(ctrl_log[2], 15, "               ");
            break;
        }
        case 1: {
            block_controls = true;
            std::snprintf(ctrl_log[0], 15, "%cside: %s      ", pointer_index == 0 ? '>' : ' ', alliance ? "RED " : "BLUE");
            std::snprintf(ctrl_log[1], 15, "%ccs: %s       ", pointer_index == 1 ? '>' : ' ', color_sort_enabled ? "ON " : "OFF");
            std::snprintf(ctrl_log[2], 15, "%cmore...      ", pointer_index == 2 ? '>' : ' ');

            if (pointer_index >= 3) pointer_index = 2;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        alliance = !alliance;
                        break;
                    }
                    case 1: {
                        color_sort_enabled = !color_sort_enabled;
                        break;
                    }
                    case 2: {
                        menu_page = 2;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 0;
            }

            break;
        }
        case 2: {
            block_controls = true;
            std::snprintf(ctrl_log[0], 15, "%cdebug...     ", pointer_index == 0 ? '>' : ' ');
            std::snprintf(ctrl_log[1], 15, "%c             ", pointer_index == 1 ? '>' : ' ');
            std::snprintf(ctrl_log[2], 15, "%c             ", pointer_index == 2 ? '>' : ' ');

            if (pointer_index >= 3) pointer_index = 2;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        menu_page = 333000;
                        break;
                    }
                    case 1: {
                        color_sort_enabled = !color_sort_enabled;
                        break;
                    }
                    case 2: {
                        menu_page = 2;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 1;
            }

            break;
        }
    }

    if (pros::millis() >= next_display_time) {
        master.print(next_display_index, 0, ctrl_log[next_display_index]);
        next_display_time = pros::millis() + 50;

        ++next_display_index;
        if (next_display_index >= 3) {
            master.clear();
            next_display_index = 0;
        }
    }
}

void async_update([[maybe_unused]] void *param) {
    printf("%sasync update task started\n", chisel::prefix().c_str());

    while (true) {
        // pid updates
        arm_pos.store(arm.get_position());
        pid_handle_process(arm_pid);
        driver_arm_command.power = arm_pid_output.load();

        // motor interfact updates
        intake_itf.clean_commands();
        intake_itf.update();
        intake_itf.push_update();
        arm_itf.clean_commands();
        arm_itf.update();
        arm_itf.push_update();

        menu_update();

        chisel::wait(PROCESS_DELAY);
    }
}

bool init_done = false;

void init() {
    printf("%sinit start\n", chisel::prefix().c_str());

    if (init_done) {
        printf("%sinit already complete\n", chisel::prefix().c_str());
        return;
    }

    init_done = true;

    (void) left_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);
    (void) right_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);

    optical.set_integration_time(PROCESS_DELAY);

    pros::lcd::initialize();
    master.clear();

    printf("%sstarting async update task\n", chisel::prefix().c_str());

    (void) arm.set_zero_position(0);

    pros::Task async_update_task(async_update);

    printf("%sstandard init complete\n", chisel::prefix().c_str());

    chassis.initialize();

    printf("%sinit complete\n", chisel::prefix().c_str());
}

void initialize() {
    printf("%sdefault init start, calling init\n", chisel::prefix().c_str());
    init();
}

void competition_initialize() {
    printf("%scompetition init start, calling init\n", chisel::prefix().c_str());
    init();
}

void autonomous() {
    printf("%sauton start\n", chisel::prefix().c_str());
}

void opcontrol() {
    printf("%sopcontrol start\n", chisel::prefix().c_str());

    intake_itf.assign_command(&driver_intake_command);
    arm_itf.assign_command(&driver_arm_command);

    while (true) {
        // drive
        if (!block_movement) {
            int32_t lateral_move = chassis.lateral_drive_settings->drive_calc_power(master.get_analog(DRIVE_JOYSTICK));
            int32_t angular_move = chassis.angular_drive_settings->drive_calc_power(master.get_analog(TURN_JOYSTICK));

            lateral_move = chisel::clamp(lateral_move, -127L, 127L);
            angular_move = chisel::clamp(angular_move, -127L, 127L);

            int32_t scaled_angular_move = angular_move * (0.7f + 0.3f * (lateral_move / 127.0f));

            (void) left_motors.move(lateral_move + scaled_angular_move);
            (void) right_motors.move(lateral_move - scaled_angular_move);
        }

        if (!block_controls) {
            // intake
            if (master.get_digital(INTAKE_FWD_BUTTON)) {
                driver_intake_command.power = 127;
            } else if (master.get_digital(INTAKE_REV_BUTTON)) {
                driver_intake_command.power = -127;
            } else {
                driver_intake_command.power = 0;
            }

            // arm
            arm_macro_cycle_toggle.tick(master.get_digital(ARM_MACRO_CYCLE_BUTTON));
            if (arm_macro_cycle_toggle.value) {
                arm_macro_cycle_toggle.value = false;

                ++arm_macro_cycle_index;
                if (arm_macro_cycle_index >= 3) {
                    arm_macro_cycle_index = 1;
                }
            }

            switch (arm_macro_cycle_index) {
                case 1: {
                    arm_target_pos.store(ARM_LOAD_POS);
                    break;
                }
                case 2: {
                    arm_target_pos.store(ARM_SCORE_POS);
                    break;
                }
            }

            if (master.get_digital(ARM_UP_BUTTON)) {
                arm_target_pos.store(arm_pos.load() + ARM_SPEED);
                arm_macro_cycle_index = 0;
            } else if (master.get_digital(ARM_DOWN_BUTTON)) {
                arm_target_pos.store(arm_pos.load() - ARM_SPEED);
                arm_macro_cycle_index = 0;
            }
            arm_target_pos.store(chisel::clamp(arm_target_pos.load(), 0.0f, MAX_ARM_POS));

            // mogo
            mogo_toggle.tick(master.get_digital(MOGO_TOGGLE_BUTTON));
            (void) mogo.set_value(mogo_toggle.value);

            // doinker
            doinker_toggle.tick(master.get_digital(DOINKER_TOGGLE_BUTTON));
            (void) doinker.set_value(doinker_toggle.value);
        }

        chisel::wait(PROCESS_DELAY);
    }
}
