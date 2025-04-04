#pragma once

#include "chisel/data/pose.h"
#include "chisel/util/util.h"

namespace chisel {

struct ExitCondition {
    float range;
    uint32_t time;

    ExitCondition(float range, uint32_t time);

    [[nodiscard]] bool get_exit() const;

    void update(float error);

    void reset();

private:
    uint32_t start_time = UINT32_MAX;
    bool exit = false;
};

class Motion {
public:
    Pose* curr_pose;

    uint32_t life;
    bool async;

    float min_speed;
    float max_speed;

    std::pair<float, float> controls;

    Motion(Pose* pose, uint32_t life, bool async, float min_speed = 0, float max_speed = 127);

    virtual void update() = 0;

    virtual void push_controls() = 0;

    virtual std::pair<float, float> get_controls();

    virtual ~Motion() = default;
};

class TurnToHeading final : public Motion {
public:
    float target_heading;

    TurnToHeading(Pose* pose, float target_heading, uint32_t life = 3000, bool async = false);

    void update() override;

    void push_controls() override;

    std::pair<float, float> get_controls() override;

private:
    float angular_pid_control = 0;
};

class TurnToPoint final : public Motion {
public:
    Pose target_point;

    TurnToPoint(Pose* pose, const Pose& target_point, uint32_t life = 3000, bool async = false);

    void update() override;

    void push_controls() override;

    std::pair<float, float> get_controls() override;

private:
    float angular_pid_control = 0;
};

class MoveToPoint final : public Motion {
public:
    Pose target_point;

    bool reversed;

    MoveToPoint(Pose* pose, const Pose& target_point, uint32_t life = 5000, bool async = false, bool reversed = false);

    void update() override;

    void push_controls() override;

    std::pair<float, float> get_controls() override;

private:
    std::pair<float, float> pid_controls {0, 0};
};

} // namespace chisel