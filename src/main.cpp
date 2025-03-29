#include "main.h"

#include "chisel/util/motoritf.h"

#include "chisel/chisel.h"

constexpr int32_t DT_FL_PORT = 6;
constexpr int32_t DT_ML_PORT = 7;
constexpr int32_t DT_BL_PORT = 8;

constexpr int32_t DT_FR_PORT = 3;
constexpr int32_t DT_MR_PORT = 4;
constexpr int32_t DT_BR_PORT = 5;

constexpr int32_t INTAKE_PORT = 2;
constexpr int32_t ARM_PORT = 9;
constexpr int32_t DOINKER_PORT = 21;

constexpr int32_t IMU_PORT = 21;
constexpr int32_t OPTICAL_PORT = 21;

pros::MotorGroup left_motors ({DT_FL_PORT, DT_ML_PORT, DT_BL_PORT});
pros::MotorGroup right_motors ({DT_FR_PORT, DT_MR_PORT, DT_BR_PORT});

pros::Motor intake (INTAKE_PORT);
auto intake_itf = chisel::MotorItf(&intake);
pros::Motor arm (ARM_PORT);

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

    for (int i = 0; i < INT_MAX; ++i) {
        const int32_t lateral_move = chassis.lateral_drive_settings->drive_calc_power(master.get_analog(ANALOG_LEFT_Y));
        const int32_t angular_move = chassis.angular_drive_settings->drive_calc_power(master.get_analog(ANALOG_RIGHT_X));

        (void)left_motors.move(lateral_move + angular_move);
        (void)left_motors.move(lateral_move - angular_move);

        chisel::wait(PROCESS_DELAY);
    }
}