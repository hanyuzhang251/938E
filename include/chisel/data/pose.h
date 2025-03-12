#pragma once

#include <atomic>

namespace chisel {

struct Pose {
    std::atomic<float> x;
    std::atomic<float> y;
    std::atomic<float> h;

    Pose(const float x, const float y, const float h): x(x), y(y), h(h) {}

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

    static Pose sum(const Pose &a, const Pose &b) {
        return {a.x.load() + b.x.load(), a.y.load() + b.y.load(), a.h.load() + b.h.load()};
    }
};

} // namespace chisel