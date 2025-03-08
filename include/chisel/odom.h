#pragma once

#include "main.h"
#include "util.h"
#include "config.h"
#include <atomic>

namespace chisel {

struct Pose {
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> h;

    Pose(float x_, float y_, float h_);

    auto operator()();
};

Pose solve_imu_bias(int32_t timeout);

} // namespace chisel