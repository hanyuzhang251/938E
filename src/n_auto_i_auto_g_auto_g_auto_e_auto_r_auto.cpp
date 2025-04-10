#include "main.h"
#include "chisel/chisel.h"
#include "robot_config.h"

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

#define S(dist, min_speed_, max_speed_) \
    lateral_pid_controller.max_speed = (max_speed_); \
    lateral_pid_controller.min_speed = (min_speed_); \
    target_dist.store((dist)); \
    wait_stable(lateral_pid_controller);
#define T(heading, min_speed_, max_speed_) \
    angular_pid_controller.max_speed = (max_speed_); \
    angular_pid_controller.min_speed = (min_speed_); \
    target_heading.store((heading)); \
    wait_stable(angular_pid_controller);

void somerandomauto(){
    printf("%sauton start\n", chisel::prefix().c_str());

    intake_itf.assign_command(&auton_intake_command);
    odom.internal_pose.x = 0;
    odom.internal_pose.y = 0;
    odom.internal_pose.h = 0;
    odom.pose.x = 0;
    odom.pose.y = 0;
    odom.pose.h = 0;
    odom.pose_offset.x = -52;
    odom.pose_offset.y = 36;
    odom.pose_offset.h = -25;

    (void)mogo.set_value(true);

    target_heading.store (odom.pose_offset.h);

    current_dist.store(0);
    target_dist.store(current_dist.load());
    target_heading.store(0);
    lateral_pid_controller.max_speed = 127;
    lateral_pid_controller.min_speed = 0;
    angular_pid_controller.max_speed = 100;
    angular_pid_controller.min_speed = 0;
    arm_target_pos.store(ARM_SCORE_POS + 200);
    pros::delay(700);
    arm_target_pos.store(-70);
    S(-3, 0, 93);
    T(-30, 0, 60);  
    rdoinker.set_value(true);
    pros::delay(200);
    S(-15, 0, 127);
    rdoinker.set_value(false);
    S(6, 0, 127);
    T(-60, 0, 60);
    mogo.set_value(false);
    S(-30, 0, 107 );    
    mogo.set_value(true);
    T(170, 0, 110);


    
    /*target_heading.store(690);
    wait_cross(angular_pid_controller, 689, false);
    target_dist.store(-40);
    wait_cross(lateral_pid_controller, 40);*/
}