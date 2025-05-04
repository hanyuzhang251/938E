#pragma once

#include "robot_config.h"

#include "main.hpp"

void run_neg_5p1_aut() {
    bool &ring_seen = side ? red_ring_seen : blue_ring_seen;
    pros::adi::DigitalOut &doinker = side ? rdoinker : ldoinker;
    const float multi = side ? 1 : -1;

    auton_init(124 * multi, 190); // init auton with starting heading of 125 deg, and arm holding ring at pos 190

    arm_target_pos.store(ARM_ALLIANCE_POS); // score ring on alliance
    wait(300); // delay to give arm time to score

    target_dist.fetch_add(-40); // start moving backwards

    wait_cross(lateral_pid_controller, -2.5); // wait so arm disengaged from ring
    arm_target_pos.store(ARM_LOAD_POS + 120); // reset arm to default position
    // arc movement to mogo
    target_heading.store(210 * multi);
    angular_pid_controller.max_speed = 127 / 2.0f;

    wait_stable(lateral_pid_controller);
    (void) mogo.set_value(false); // clamp the mogo
    wait(200); // delay so mogo can clamp correctly

    angular_pid_controller.max_speed = 127;

    // Heading should be -45.5 degrees if before 6:00 PM, -39 degrees after
    target_heading.store(-38.5 * multi); // Point towards 4 ring stack.

    wait_stable(angular_pid_controller, 5000, 3, 8, 2.5);

    auton_intake_command.power = 127; // start running intake
    target_dist.fetch_add(21.5); // collect closer ring

    // wait until we have collected the ring, for a maximum of 1500 ms
    uint32_t end = pros::millis() + 1500;
    ring_seen = false;
    bool prs = ring_seen;
    while (pros::millis() < end) {
        if (!ring_seen && prs) break;
        prs = ring_seen;
        wait(10);
    }
    wait(150); // delay a little extra so ring goes on mogo correctly

    wait_stable(lateral_pid_controller); // wait until we finish moving, just in case if we collect the ring early

    // shallow arc to second ring
    target_heading.store(-95 * multi); // overshoot a little towards our side to prevent crossing
    angular_pid_controller.max_speed = 127;
    target_dist.fetch_add(20);

    // // wait until we see a ring, for a maximum of 1200 ms starting after 650 ms.
    // ring_seen = false;
    // uint32_t start = pros::millis() + 650;
    // end = pros::millis() + 1200;
    // while (pros::millis() < end) {
    //     if (ring_seen && pros::millis() > start) break;
    //     prs = ring_seen;
    //     wait(10);
    // }
    // // Note the difference as here we finish waiting upon first sight of a ring, and don't wait extra.

    wait_stable(lateral_pid_controller, 2000, 3, 8); // wait until we complete the movement
    wait(150);

    // Reset speeds to max
    lateral_pid_controller.max_speed = 127;
    angular_pid_controller.max_speed = 127;

    // Arc movement to exit ring stack area without crossing
    target_dist.fetch_add(-32.5);
    wait(150);
    target_heading.store(-45 * multi);

    wait_stable(lateral_pid_controller); // wait until movement is complete.

    // Arc movement to collect sole ring stack
    target_heading.store(-135 * multi);
    target_dist.fetch_add(26);

    wait_cross(lateral_pid_controller, 15.5); // Wait until we cross where the ring stack would be

    target_heading.store(-190 * multi); // Position for movement to corner ring stack

    wait_stable(angular_pid_controller, 5000, 3, 8, 3.5); // Wait until turning is complete, with a very wide tolerance

    // Speed tooning
    lateral_pid_controller.max_speed = 127;
    angular_pid_controller.max_speed = 63;

    // Arc movement to collect corner rings
    target_dist.fetch_add(60);
    target_heading.store(-135 * multi);

    wait_cross(lateral_pid_controller, 3);

    for (int i = 1; i <= 10; ++i) {
        lateral_pid_controller.max_speed = 127 - 5 * i;
        wait(30);
    }

    unstuck_enabled = false; // disable unjammer so we don't outtake ring by accident

    wait(1250); // Wait ____ ms which is about when we have entered the ring stack

    // Back out to pull the bottom ring.
    lateral_pid_controller.max_speed = 17;
    target_dist.store(current_dist.load() - 4);
    for (int i = 1; i <= 5; ++i) {
        lateral_pid_controller.max_speed = 17 + i * 5;
        wait(40);
    }

    wait_stable(lateral_pid_controller); // wait until movement is complete

    // Move forward agian to collect the ring.

    lateral_pid_controller.max_speed = 127;
    target_dist.fetch_add(4);

    wait(500); // wait until movement is complete;

    // Back out and face ring stack closer to alliance stake
    target_dist.store(current_dist.load() - 9);
    lateral_pid_controller.max_speed = 127;
    wait_cross(lateral_pid_controller, -6);
    target_heading.store(77.5 * multi);
    angular_pid_controller.max_speed = 127;

    wait_stable(angular_pid_controller); // Wait until turning is complete.

    target_dist.fetch_add(34); // move towards alliance ring stack

    unstuck_enabled = true; // re-enable unjammer

    // When approaching ring stack, activate doinker
    wait_cross(lateral_pid_controller, 32.67);
    (void) doinker.set_value(true);

    wait_stable(lateral_pid_controller); // Wait until movement is complete

    auton_intake_command.power = 127; // Activate intake

    // Pull the ring back slowly. Due to poor doinker quality, usually doesn't pull all the way off the ring stack.
    target_dist.fetch_add(-9);
    lateral_pid_controller.max_speed = 127 / 3.0f;
    wait_stable(lateral_pid_controller);
    (void) doinker.set_value(false);

    target_heading.store(92 * multi); // Point towards ring. Overshoot a little for consistency.
    wait(300);

    // Speed settings. Carefully tuned
    lateral_pid_controller.max_speed = 127;
    angular_pid_controller.max_speed = 127 / 2.0f;

    // Arc movement to collect ring and point towards tower
    target_dist.fetch_add(33);
    wait_cross(lateral_pid_controller, 12);
    target_heading.store(-20 * multi);

    // After collecting the ring, boost it a little so we reach the tower
    wait_cross(lateral_pid_controller, 9);
    target_dist.fetch_add(15);

    // After short delay, lower arm to make contact with the tower.
    wait(350);
    arm_target_pos.store(ARM_SCORE_POS);

    // Ensure mogo stays clamped.
    mogo_toggle.value = false;

    menu_page = 0;

    wait(800);
}