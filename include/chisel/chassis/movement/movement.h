#pragma once

#include "chisel/data/pose.h"
#include "chisel/util/util.h"

namespace chisel {

class Movement {
public:
    Pose* curr_pose;

    uint32_t life;
    bool async;

    std::pair<float, float> controls;

    Movement(Pose* pose, uint32_t life, bool async);

    virtual void update() = 0;

    virtual void push_controls() = 0;

    virtual std::pair<float, float> get_controls();

    virtual ~Movement() = default;
};

class TurnToHeading final : public Movement {
public:
    float target_heading;

    TurnToHeading(Pose* pose, float target_heading, uint32_t life = 3000, bool async = false);

    void update() override;

    void push_controls() override;

private:
    float angular_pid_control = 0;
};

class TurnToPoint final : public Movement {
public:
    Pose target_point;

    TurnToPoint(Pose* pose, const Pose& target_point, uint32_t life = 3000, bool async = false);

    void update() override;

    void push_controls() override;

    std::pair<float, float> get_controls() override;

private:
    float angular_pid_control = 0;
};

} // namespace chisel