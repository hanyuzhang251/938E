#pragma once

#include "main.h"
#include <atomic>

namespace chisel {

struct Pose {                                                                  
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> h;

    Pose(float x_pos, float y_pos, float head);

    auto operator()();
};

Pose solve_imu_bias(int32_t timeout);

struct OdomProcess {
    Pose robot_pose;
    Pose robot_abs_pose;
    Pose robot_mod_pose;

    bool using_ime;
    pros::MotorGroup* left_ime;
    pros::MotorGroup* right_ime;
    float left_ime_prev_pos;
    float right_ime_prev_pos;

    bool using_v_odom;
    pros::Rotation* v_odom;
    float v_odom_prev_pos;
    float v_odom_h_offset;
    float v_odom_v_offset;

    bool using_h_odom;
    pros::Rotation* h_odom;
    float h_odom_prev_pose;
    float h_odom_h_offset;
    float h_odom_v_offset;

    bool using_inertial;
    pros::Imu* inertial; 

    OdomProcess(Pose r_pose, Pose r_abs_pose, Pose r_mod_pose,
        pros::MotorGroup* l_ime, pros::MotorGroup* r_ime,
        pros::Rotation* v_od, pros::Rotation* h_od,
        pros::Imu* imu);

    auto operator()();
};

void odom_handle_process(OdomProcess& process);

} // namespace chisel