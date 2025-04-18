#include "main.h"
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

    if (optical.get_proximity() < 200) return;

    const double r_hue_min = std::fmod(RED_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double r_hue_max = std::fmod(RED_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);
    const double b_hue_min = std::fmod(BLUE_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double b_hue_max = std::fmod(BLUE_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);

    bool b_ring = false;

    if (b_hue_min <= b_hue_max) {
        // if it's normal, just check the ranges
        b_ring = b_hue_min <= optical.get_hue() && optical.get_hue() <= b_hue_max;
    } else if (b_hue_min >= b_hue_max) {
        // if the range goes around 0, say hue min is 330 while hue_max is 30,
        // 0 and 360 are used as bounds.
        b_ring = optical.get_hue() <= b_hue_max || optical.get_hue() >= b_hue_min;
    }
    if (b_ring) blue_ring_seen = true;

    bool r_ring = false;
    if (r_hue_min <= r_hue_max) {
        // if it's normal, just check the ranges
        r_ring = r_hue_min <= optical.get_hue() && optical.get_hue() <= r_hue_max;
    } else if (r_hue_min >= r_hue_max) {
        // if the range goes around 0, say hue min is 330 while hue_max is 30,
        // 0 and 360 are used as bounds.
        r_ring = optical.get_hue() <= r_hue_max || optical.get_hue() >= r_hue_min;
    }
    if (r_ring) red_ring_seen = true;

    if (pros::millis() - last_outtake >= COLOR_SORT_COOLDOWN && (alliance && !b_ring && p_brs || !alliance && !r_ring && p_rrs)) {
        pros::Task([] { queue_outtake(); });
        last_outtake = pros::millis();
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
            std::snprintf(ctrl_log[0], 15, "%cside: %s      ", pointer_index == 0 ? '>' : ' ',
                          alliance ? "RED " : "BLUE");
            std::snprintf(ctrl_log[1], 15, "%ccs: %s       ", pointer_index == 1 ? '>' : ' ',
                          color_sort_enabled ? "ON " : "OFF");
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

static void wait_stable(const chisel::PIDController &pid_process, const uint32_t timeout = 5000, const int buffer_ticks = 3,
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

static void wait_cross(const chisel::PIDController &pid_process, float point, const bool relative = true, const int buffer_ticks = 0) {
    if (relative) point += pid_process.value.load();

   const bool side = pid_process.value.load() >= point;
    while ((pid_process.value.load() >= point) == side) {
        wait(PROCESS_DELAY);
    }
    for (int i = 0; i < buffer_ticks; ++i) wait(PROCESS_DELAY);
}

//#include "bezier_motion.hpp"

void autonomous() {
    /*printf("%sauton start\n", chisel::prefix().c_str());

    intake_itf.assign_command(&auton_intake_command);

    const float multi = alliance ? 1 : -1;

    odom.internal_pose.x = 0;
    odom.internal_pose.y = 0;
    odom.internal_pose.h = 0;
    odom.pose.x = 0;
    odom.pose.y = 0;
    odom.pose.h = 0;
    odom.pose_offset.x = -52;
    odom.pose_offset.y = 36;
    odom.pose_offset.h = -25 * multi;

    (void)mogo.set_value(true);

    target_heading.store (odom.pose_offset.h);

    current_dist.store(0);
    target_dist.store(current_dist.load());
    chassis.state = AUTON_STATE;
    menu_page = 333100;
    pointer_index = 0;
    wait(15);

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

    auton_intake_command.power = 127;

    target_dist.fetch_add(47);
    (void)rdoinker.set_value(true);

    wait_stable(lateral_pid_controller);

    target_dist.fetch_add(-30);

    pros::Task([&] {
        const uint32_t end = pros::millis() + 1200;
        while (!red_ring_seen && pros::millis() < end) {
            wait(PROCESS_DELAY / 2);
        }
        auton_intake_command.power = 0;
    });

    wait_cross(lateral_pid_controller, -2);
    target_heading.store(-80 * multi);

    wait_stable(lateral_pid_controller);

    (void)mogo.set_value(false);
    (void)rdoinker.set_value(false);
    wait(250);

    auton_intake_command.power = 127;

    target_heading.store(-60 * multi);
    wait_stable(angular_pid_controller);

    target_dist.fetch_add(33);

    wait_cross(lateral_pid_controller, 15);
    target_heading.store(-90 * multi);

    wait_cross(lateral_pid_controller, 29);
    arm_target_pos.store(ARM_LOAD_POS);
    wait_stable(lateral_pid_controller);

    wait(3000);

    chassis.state = DRIVE_STATE;*/
    //left_motors.move(127);
	//target_dist.fetch_add(90);
    /*chassis.state = AUTON_STATE;
    
	//target_dist.store(30);
	//wait_stable(lateral_pid_controller);
    bezier_motion::bezier(bezier_motion::vec2(51, 40), 0.5, bezier_motion::vec2(0.76, 0.76))
        .execute_pid_motion(NO_PREVIOUS_CONTROL_POINT, [](double lin_dist){
            lateral_pid_controller.max_speed = 127;
            lateral_pid_controller.min_speed = 6;
            target_dist.store(lin_dist);
            wait_stable(lateral_pid_controller);
        }, [](double ang_offset_heading){
            angular_pid_controller.max_speed = 127;
            angular_pid_controller.min_speed = 6;
            target_heading.store(ang_offset_heading);
            wait_stable(angular_pid_controller);
        });*/
    chassis.state = AUTON_STATE;
    extern void somerandomauto(); 
    somerandomauto();
    chassis.state = DRIVE_STATE;

}

void opcontrol() {
    printf("%sopcontrol start\n", chisel::prefix().c_str());

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
