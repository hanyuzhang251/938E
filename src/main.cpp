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
constexpr int32_t DOINKER_PORT = 21;

constexpr int32_t IMU_PORT = 21;
constexpr int32_t OPTICAL_PORT = 21;

constexpr pros::anlg_button DRIVE_JOYSTICK = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr pros::anlg_button TURN_JOYSTICK = pros::E_CONTROLLER_ANALOG_RIGHT_X;

constexpr pros::digi_button INTAKE_FWD_BUTTON = pros::E_CONTROLLER_DIGITAL_L1;
constexpr pros::digi_button INTAKE_REV_BUTTON = pros::E_CONTROLLER_DIGITAL_L2;

constexpr pros::digi_button DOINKER_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_X;
chisel::Toggle doinker_toggle (false);

constexpr pros::digi_button MOGO_TOGGLE_BUTTON = pros::E_CONTROLLER_DIGITAL_A;
chisel::Toggle mogo_toggle (false);

constexpr pros::digi_button ARM_UP_BUTTON = pros::E_CONTROLLER_DIGITAL_R1;
constexpr pros::digi_button ARM_DOWN_BUTTON = pros::E_CONTROLLER_DIGITAL_R2;

constexpr pros::digi_button ARM_MACRO_CYCLE_BUTTON = pros::E_CONTROLLER_DIGITAL_UP;
chisel::Toggle arm_macro_cycle_toggle (false);

pros::MotorGroup left_motors ({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
pros::MotorGroup right_motors ({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

pros::Motor intake (INTAKE_PORT);
auto intake_itf = chisel::MotorItf(&intake);
chisel::Command driver_intake_command = {0, 1};

pros::Motor arm (ARM_PORT);
auto arm_itf = chisel::MotorItf(&arm);
chisel::Command driver_arm_command = {0, 1};

constexpr float ARM_SPEED = 220;

constexpr float MAX_ARM_POS = 2000;
constexpr float ARM_LOAD_POS = 250;
constexpr float ARM_SCORE_POS = 800;

int arm_macro_cycle_index = 0;

chisel::PIDSettings arm_pid_settings {
    0.6, 0.1, 3, 70, 999, 999, 0, 0, 0
};
std::atomic<float> arm_pos (0);
std::atomic<float> arm_target_pos (0);
std::atomic<float> arm_pid_output (0);
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

pros::Imu imu (IMU_PORT);
pros::Optical optical (OPTICAL_PORT);

chisel::DriveTrain drive_train (
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

chisel::Odom odom {
            {0, 0, 0},
            {0, 0, 0},
            &imu,
            &drive_train,
            nullptr,
            0
        };

chisel::PIDSettings angular_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::PIDSettings lateral_pid_settings {
    10, 0, 0, 0, 0, 999, 0, 0, 0
};

chisel::Chassis chassis = {&drive_train, &lateral_drive_settings, &angular_drive_settings, &odom, &angular_pid_settings, &lateral_pid_settings, true};

pros::Controller master(pros::E_CONTROLLER_MASTER);

char ctrl_log[3][15];
uint32_t next_display_time = pros::millis();
int next_display_index = 0;
void menu_update() {
    std::snprintf(ctrl_log[0], 15, "ut: %s", chisel::prefix().substr(1,5).c_str());

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

void async_update([[maybe_unused]] void* param) {
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

    (void)left_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);
    (void)right_motors.set_brake_mode_all(MOTOR_BRAKE_COAST);

    optical.set_integration_time(PROCESS_DELAY);

    pros::lcd::initialize();
    master.clear();

    printf("%sstarting async update task\n", chisel::prefix().c_str());

    (void)arm.set_zero_position(0);

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

    while(true) {
        // drive
        int32_t lateral_move = chassis.lateral_drive_settings->drive_calc_power(master.get_analog(DRIVE_JOYSTICK));
        int32_t angular_move = chassis.angular_drive_settings->drive_calc_power(master.get_analog(TURN_JOYSTICK));

        lateral_move = chisel::clamp(lateral_move, -127L, 127L);
        angular_move = chisel::clamp(angular_move, -127L, 127L);

        int32_t scaled_angular_move = angular_move * (0.7f + 0.3f * (lateral_move / 127.0f));

        (void)left_motors.move(lateral_move + scaled_angular_move);
        (void)right_motors.move(lateral_move - scaled_angular_move);

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

        switch(arm_macro_cycle_index) {
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
        chisel::wait(PROCESS_DELAY);

        printf("%samci:%d\n", chisel::prefix().c_str(), arm_macro_cycle_index);
    }
}