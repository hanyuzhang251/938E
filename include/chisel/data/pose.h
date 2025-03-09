#pragma once

#include <atomic>

namespace chisel {

struct Pose {
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> h;

    Pose(float x, float y, float h): x(x), y(y), h(h) {}

    Pose(const Pose &other): x(other.x.load()), y(other.y.load()), h(other.h.load()) {}

    Pose& operator=(const Pose &other) {
        if (this != &other) {
            x.store(other.x.load());
            y.store(other.y.load());
            h.store(other.h.load());
        }
        return *this;
    }
    
    auto operator()() {
        return std::tie(x, y, h);
    };
};

} // namespace chisel