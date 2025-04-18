#pragma once

#define MOVE_FORWARD(dist, min_speed, max_speed) \
    lateral_pid_controller.max_speed = (max_speed); \
    lateral_pid_controller.min_speed = (min_speed); \
    target_dist.store((dist)); \
    wait_stable(lateral_pid_controller, (dist));