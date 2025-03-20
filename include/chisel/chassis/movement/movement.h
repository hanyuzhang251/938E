#pragma once

#include "chisel/data/pose.h"

namespace chisel {

class Movement {
public:
    virtual void setup(Pose dest, bool async, int n_args, float* args) = 0;
    virtual bool is_async() = 0;

    virtual void pull_pose(Pose new_pose) = 0;

    virtual void update() = 0;
    virtual std::pair<float, float> get_controls();

    virtual ~Movement() = default;
};

} // namespace chisel