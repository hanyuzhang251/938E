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

    Pose(float x_pos, float y_pos, float head);

    Pose(const Pose& other)
        : x(other.x.load()), y(other.y.load()), h(other.h.load()) {}

    Pose& operator=(const Pose& other) {
        if (this != &other) {
            x.store(other.x.load());
            y.store(other.y.load());
            h.store(other.h.load());
        }
        return *this;
    }

    auto operator()();
};

Pose solve_imu_bias(int32_t timeout);

} // namespace chisel