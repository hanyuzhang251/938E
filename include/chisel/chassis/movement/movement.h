#pragma once

#include "chisel/data/pose.h"

namespace chisel {

class Movement {
public:
    bool async;
    uint32_t life;

    Movement(const bool async, const uint32_t life) : async(async), life(life) {}

    virtual void setup() = 0;

    virtual ~Movement() = default;
};

} // namespace chisel