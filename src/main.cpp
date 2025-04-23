#include "main.h"

#include <csetjmp>

#include "chisel/chisel.h"

#include "robot_config.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

bool alliance = true; // true = red, false = blue
bool color_sort_enabled = true;

bool red_ring_seen = false;
bool blue_ring_seen = false;

uint32_t last_outtake = 0;

void queue_outtake() {
    wait(100);

    color_sort_command.power = -30;
    color_sort_command.priority = 999;

    wait(200);

    color_sort_command.priority = 0;
}

void color_sort_update() {
    const bool p_brs = blue_ring_seen;
    const bool p_rrs = red_ring_seen;

    blue_ring_seen = false;
    red_ring_seen = false;

    if (!color_sort_enabled) return;

    const double r_hue_min = std::fmod(RED_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double r_hue_max = std::fmod(RED_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);
    const double b_hue_min = std::fmod(BLUE_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double b_hue_max = std::fmod(BLUE_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);

    bool b_ring = false;
    bool r_ring = false;

    if (optical.get_proximity() >= 200) {
        if (b_hue_min <= b_hue_max) {
            // if it's normal, just check the ranges
            b_ring = b_hue_min <= optical.get_hue() && optical.get_hue() <= b_hue_max;
        } else if (b_hue_min >= b_hue_max) {
            // if the range goes around 0, say hue min is 330 while hue_max is 30,
            // 0 and 360 are used as bounds.
            b_ring = optical.get_hue() <= b_hue_max || optical.get_hue() >= b_hue_min;
        }
        if (b_ring) blue_ring_seen = true;

        if (r_hue_min <= r_hue_max) {
            // if it's normal, just check the ranges
            r_ring = r_hue_min <= optical.get_hue() && optical.get_hue() <= r_hue_max;
        } else if (r_hue_min >= r_hue_max) {
            // if the range goes around 0, say hue min is 330 while hue_max is 30,
            // 0 and 360 are used as bounds.
            r_ring = optical.get_hue() <= r_hue_max || optical.get_hue() >= r_hue_min;
        }
        if (r_ring) red_ring_seen = true;
    }

    if (pros::millis() - last_outtake >= COLOR_SORT_COOLDOWN) {
        if ((alliance && !b_ring && p_brs || !alliance && !r_ring && p_rrs)) {
            pros::Task([] {
                queue_outtake();
                last_outtake = pros::millis();
            });
        }
    }
}

bool block_movement = false;
bool block_controls = false;

char ctrl_log[3][15];
int menu_page = 0;
int pointer_index = 0;

uint32_t next_display_time = pros::millis();
int next_display_index = 0;

void menu_update() {
    menu_toggle.tick(master.get_digital(MENU_TOGGLE_BUTTON));
    if (menu_toggle.value && menu_page != 254000) {
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
    }

    bool menu_back = false;
    menu_back_toggle.tick(master.get_digital(MENU_BACK_BUTTON));
    if (menu_back_toggle.value) {
        menu_back_toggle.value = false;
        menu_back = true;
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
            if (pointer_index == 3) {
                std::snprintf(ctrl_log[0], 15, "%cmore...      ", pointer_index == 3 ? '>' : ' ');
                std::snprintf(ctrl_log[1], 15, "               ");
                std::snprintf(ctrl_log[2], 15, "               ");
            } else {
                std::snprintf(ctrl_log[0], 15, "%creset_arm    ", pointer_index == 0 ? '>' : ' ');
                std::snprintf(ctrl_log[1], 15, "%cside: %s      ", pointer_index == 1 ? '>' : ' ',
                              alliance ? "RED " : "BLUE");
                std::snprintf(ctrl_log[2], 15, "%ccs: %s       ", pointer_index == 2 ? '>' : ' ',
                              color_sort_enabled ? "ON " : "OFF");
            }

            if (pointer_index >= 4) pointer_index = 3;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        pros::Task([&] {
                            const bool p_arm_clamp = arm_clamp;
                            arm_clamp = false;

                            arm_target_pos.fetch_add(-9999);

                            while (arm.get_current_draw() < 2300) {
                                chisel::wait(PROCESS_DELAY);
                            }

                            reset_arm();

                            arm_clamp = p_arm_clamp;
                        });

                        menu_toggle.tick(true);
                        menu_toggle.tick(false);

                        break;
                    }
                    case 1: {
                        alliance = !alliance;
                        break;
                    }
                    case 2: {
                        color_sort_enabled = !color_sort_enabled;
                        break;
                    }
                    case 3: {
                        menu_page = 2;
                        pointer_index = 0;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 0;
                pointer_index = 0;
            }

            break;
        }
        case 2: {
            block_controls = true;
            std::snprintf(ctrl_log[0], 15, "%crepair...    ", pointer_index == 0 ? '>' : ' ');
            std::snprintf(ctrl_log[1], 15, "%cdebug...     ", pointer_index == 1 ? '>' : ' ');
            std::snprintf(ctrl_log[2], 15, "               ");

            if (pointer_index >= 2) pointer_index = 1;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        menu_page = 409000;
                        pointer_index = 0;
                        break;
                    }
                    case 1: {
                        menu_page = 333000;
                        pointer_index = 0;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 1;
                pointer_index = 0;
            }

            break;
        }
        case 409000: {
            block_controls = true;
            std::snprintf(ctrl_log[0], 15, "%carm...       ", pointer_index == 0 ? '>' : ' ');
            std::snprintf(ctrl_log[1], 15, "               ");
            std::snprintf(ctrl_log[2], 15, "               ");

            if (pointer_index >= 1) pointer_index = 0;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        menu_page = 409100;
                        pointer_index = 0;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 1;
                pointer_index = 0;
            }

            break;
        }
        case 409100: {
            block_controls = false;
            std::snprintf(ctrl_log[0], 15, "%ca_clamp: %s   ", pointer_index == 0 ? '>' : ' ',
                          arm_clamp ? "ON " : "OFF");
            std::snprintf(ctrl_log[1], 15, "%c%c a_tare_pos ", pointer_index == 1 ? '>' : ' ',
                          arm_pos.load() > -5 && arm_pos.load() < 5 ? 'O' : '@');
            std::snprintf(ctrl_log[2], 15, "%c%c a_q-reset  ", pointer_index == 2 ? '>' : ' ',
                          arm_pos.load() > -5 && arm_pos.load() < 5 ? 'O' : '@');

            if (pointer_index >= 3) pointer_index = 2;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        arm_clamp = !arm_clamp;
                        break;
                    }
                    case 1: {
                        pros::Task reset_arm_task([] { reset_arm(); });
                        pointer_index = 0;
                        break;
                    }
                    case 2: {
                        pros::Task([&] {
                            const bool p_arm_clamp = arm_clamp;
                            arm_clamp = false;

                            arm_target_pos.fetch_add(-9999);

                            while (arm.get_current_draw() < 2300) {
                                chisel::wait(PROCESS_DELAY);
                            }

                            reset_arm();

                            arm_clamp = p_arm_clamp;
                        });
                    }
                }
            }

            if (menu_back) {
                menu_page = 409000;
                pointer_index = 0;
            }

            break;
        }
        case 333000: {
            block_controls = true;
            std::snprintf(ctrl_log[0], 15, "%cpos_track... ", pointer_index == 0 ? '>' : ' ');
            std::snprintf(ctrl_log[1], 15, "%carm...       ", pointer_index == 1 ? '>' : ' ');
            std::snprintf(ctrl_log[2], 15, "%coptical...   ", pointer_index == 2 ? '>' : ' ');

            if (pointer_index >= 3) pointer_index = 2;

            if (menu_select) {
                switch (pointer_index) {
                    case 0: {
                        menu_page = 333100;
                        pointer_index = 0;
                        break;
                    }
                    case 1: {
                        menu_page = 333200;
                        pointer_index = 0;
                        break;
                    }
                    case 2: {
                        menu_page = 333300;
                        pointer_index = 0;
                        break;
                    }
                }
            }

            if (menu_back) {
                menu_page = 2;
                pointer_index = 0;
            }

            break;
        }
        case 333100: {
            block_controls = true;
            if (pointer_index == 0) {
                std::snprintf(ctrl_log[0], 15, "x_pos:%f       ", chassis.odom->pose.x.load());
                std::snprintf(ctrl_log[1], 15, "y_pos:%f       ", chassis.odom->pose.y.load());
                std::snprintf(ctrl_log[2], 15, "head:%f        ", chassis.odom->pose.h.load());
            } else if (pointer_index == 1) {
                std::snprintf(ctrl_log[0], 15, "dist:%f        ", current_dist.load());
                std::snprintf(ctrl_log[1], 15, "               ");
                std::snprintf(ctrl_log[2], 15, "               ");
            }

            if (pointer_index >= 2) pointer_index = 1;

            if (menu_back) {
                menu_page = 333000;
                pointer_index = 0;
            }

            break;
        }
        case 333200: {
            block_controls = false;
            std::snprintf(ctrl_log[0], 15, "pos:%f         ", arm_pos.load());
            std::snprintf(ctrl_log[1], 15, "t_pos:%f       ", arm_target_pos.load());
            std::snprintf(ctrl_log[2], 15, "c_draw:%ld     ", arm.get_current_draw());

            if (menu_back) {
                menu_page = 333000;
                pointer_index = 0;
            }

            break;
        }
        case 333300: {
            block_controls = false;
            if (pointer_index == 0) {
                std::snprintf(ctrl_log[0], 15, "hue:%d         ", static_cast<int>(std::round(optical.get_hue())));
                std::snprintf(ctrl_log[1], 15, "dist:%ld       ", optical.get_proximity());
                std::snprintf(ctrl_log[2], 15, "int-time:%f    ", optical.get_integration_time());
            } else if (pointer_index == 1) {
                std::snprintf(ctrl_log[0], 15, "r-seen:%s      ", red_ring_seen ? "YES" : "NO ");
                std::snprintf(ctrl_log[1], 15, "b-seen:%s      ", blue_ring_seen ? "YES" : "NO ");
                std::snprintf(ctrl_log[2], 15, "               ");
            }

            if (pointer_index >= 2) pointer_index = 1;

            if (menu_back) {
                menu_page = 333000;
                pointer_index = 0;
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
        color_sort_update();

        device_update();

        menu_update();

        wait(PROCESS_DELAY);
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

    device_init();

    pros::lcd::initialize();
    master.clear();

    printf("%sstarting async update task\n", chisel::prefix().c_str());

    (void) arm.set_zero_position(0);

    pros::Task async_update_task(async_update);

    printf("%sstandard init complete\n", chisel::prefix().c_str());

    chassis.initialize();
    chassis.enabled.store(true);

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

void wait_stable(const chisel::PIDController &pid_process, const uint32_t timeout = 5000, const int buffer_ticks = 3,
                 const int min_stable_ticks = 8) {
    int stable_ticks = 0;
    const uint32_t end_time = pros::millis() + timeout;

    while (stable_ticks < min_stable_ticks) {
        pros::lcd::print(4, "error: %f", pid_process.get_error());
        if (std::abs(pid_process.get_error()) <= pid_process.pid.tolerance) ++stable_ticks;
        else stable_ticks = 0;

        if (pros::millis() >= end_time) return;
        wait(PROCESS_DELAY);
    }
    for (int i = 0; i < buffer_ticks; ++i) wait(PROCESS_DELAY);
}

void wait_cross(const chisel::PIDController &pid_process, float point, const bool relative = true, const int buffer_ticks = 0) {
    if (relative) point += pid_process.value.load();

   const bool side = pid_process.value.load() >= point;
    while ((pid_process.value.load() >= point) == side) {
        wait(PROCESS_DELAY);
    }
    for (int i = 0; i < buffer_ticks; ++i) wait(PROCESS_DELAY);
}

void red_neg_12_aut() {
    auton_init(124, 190); // init auton with starting heading of 125 deg, and arm holding ring at pos 190

    arm_target_pos.store(ARM_ALLIANCE_POS); // score ring on alliance
    wait(300); // delay to give arm time to score

    target_dist.fetch_add(-40); // start moving backwards

    wait_cross(lateral_pid_controller, -2.5); // wait so arm disengaged from ring
    arm_target_pos.store(-50); // reset arm to default position
    // arc movement to mogo
    target_heading.store(210);
    angular_pid_controller.max_speed = 127 / 2.0f;

    wait_stable(lateral_pid_controller);
    (void)mogo.set_value(false); // clamp the mogo
    wait(200);

    angular_pid_controller.max_speed = 127;
    target_heading.store(-39);
    wait_stable(angular_pid_controller);

    auton_intake_command.power = 127;
    target_dist.fetch_add(21.5);

    uint32_t end = pros::millis() + 800;
    while (pros::millis() < end) {
        const bool prs = red_ring_seen;
        if (!red_ring_seen && prs) break;
        wait(10);
    }

    wait_stable(lateral_pid_controller);

    target_heading.store(-95);
    angular_pid_controller.max_speed = 127;
    target_dist.fetch_add(18);

    red_ring_seen = false;
    uint32_t start = pros::millis() + 650;
    end = pros::millis() + 1500;
    while (pros::millis() < end) {
        const bool prs = red_ring_seen;
        if (red_ring_seen && pros::millis() > start) break;
        wait(10);
    }

    target_dist.fetch_add(-28.5);
    lateral_pid_controller.max_speed = 127;
    wait(100);
    target_heading.store(-45);
    angular_pid_controller.max_speed = 127;


    wait_stable(lateral_pid_controller);

    target_heading.store(-135);
    angular_pid_controller.max_speed = 127;
    target_dist.fetch_add(24);
    lateral_pid_controller.max_speed = 127;

    wait_cross(lateral_pid_controller, 14);

    target_heading.store(-190);

    wait_stable(angular_pid_controller);
    wait(250);

    lateral_pid_controller.max_speed = 60;
    angular_pid_controller.max_speed = 30;

    target_dist.fetch_add(43);
    wait_cross(lateral_pid_controller, 7.5f);
    target_heading.store(-135);

    wait(1250);

    lateral_pid_controller.max_speed = 15;
    target_dist.store(current_dist.load() - 4);
    wait_stable(lateral_pid_controller);

    wait(200);

    red_ring_seen = false;
    start = pros::millis() + 300;
    end = pros::millis() + 1500;
    while (pros::millis() < end) {
        if (red_ring_seen && pros::millis() > start) {
            auton_intake_command.power = 0;
            break;
        }
        wait(10);
    }

    lateral_pid_controller.max_speed = 127;
    target_dist.fetch_add(4);
    wait(500);

    target_dist.store(current_dist.load() - 4);
    target_heading.store(73);
    angular_pid_controller.max_speed = 127;
    wait_stable(angular_pid_controller);

    target_dist.fetch_add(43);
    wait_cross(lateral_pid_controller, 40);

    (void)rdoinker.set_value(true);

    wait_stable(lateral_pid_controller);

    lateral_pid_controller.max_speed = 127 / 3.0f;

    auton_intake_command.power = 127;

    target_dist.fetch_add(-9);
    wait_stable(lateral_pid_controller);
    (void)rdoinker.set_value(false);

    target_heading.store(95);
    lateral_pid_controller.max_speed = 127;
    wait(300);

    target_dist.fetch_add(33);
    wait_cross(lateral_pid_controller, 12);

    target_heading.store(-20);
    angular_pid_controller.max_speed = 127 / 2.0f;

    wait_cross(lateral_pid_controller, 9);
    target_dist.fetch_add(15);
    wait(250);
    arm_target_pos.store(ARM_SCORE_POS);

    wait(800);
}

void autonomous() {
    printf("%sauton start\n", chisel::prefix().c_str());

    menu_page = 333100;
    pointer_index = 0;

    red_neg_12_aut();

    chassis.state = DRIVE_STATE;
    wait(15);

    (void)left_motors.set_brake_mode_all(pros::MotorBrake::coast);
    (void)right_motors.set_brake_mode_all(pros::MotorBrake::coast);

    (void)left_motors.move(0);
    (void)right_motors.move(0);

    auton_intake_command.priority = 0;
    auton_intake_command.dismiss();
}

void opcontrol() {
    printf("%sopcontrol start\n", chisel::prefix().c_str());
    block_movement = false;
    block_controls = false;

    auton_intake_command.power = 0;
    auton_intake_command.priority = -999;
    auton_intake_command.dismiss();

    intake_itf.assign_command(&driver_intake_command);

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
            if (arm_clamp)
                arm_target_pos.store(chisel::clamp(arm_target_pos.load(), 0.0f, MAX_ARM_POS));

            // mogo
            mogo_toggle.tick(master.get_digital(MOGO_TOGGLE_BUTTON));
            (void) mogo.set_value(mogo_toggle.value);

            // doinker
            ldoinker_toggle.tick(master.get_digital(LDOINKER_TOGGLE_BUTTON));
            (void) ldoinker.set_value(ldoinker_toggle.value);

            rdoinker_toggle.tick(master.get_digital(RDOINKER_TOGGLE_BUTTON));
            (void) rdoinker.set_value(rdoinker_toggle.value);
        }

        wait(PROCESS_DELAY);
    }
}
