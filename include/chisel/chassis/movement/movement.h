#pragma once

#include "chisel/data/pose.h"
#include "chisel/util/util.h"

namespace chisel {

    /**
    * @brief Exit condition for motions.
    */
    struct ExitCondition {
        float range;
        uint32_t time;

        /**
         * @brief Exit condition constructor.
         *
         * @param range Absolute range of error to exit.
         * @param time Milliseconds error has to be in the range to exit.
         */
        ExitCondition(float range, uint32_t time);

        /**
         * @return Whether the exit condition is met.
         */
        [[nodiscard]] bool get_exit() const;

        /**
         * @brief Updates exit condition
         *
         * @param error Current error.
         */
        void update(float error);

        /**
         * @brief Resets the exit condition
         */
        void reset();

    private:
        uint32_t start_time = UINT32_MAX;
        bool exit = false;
    };

    /**
     * Abstract base motion class
     */
    class Motion {
    public:
        Pose* curr_pose;

        uint32_t life;
        bool async;

        float min_speed;
        float max_speed;

        ExitCondition lateral_exit;
        ExitCondition angular_exit;

        std::pair<float, float> controls;

        /**
         * @brief Motion constructor
         *
         * @param pose Pointer to the current pose
         * @param min_speed Absolute minimum speed for the motion
         * @param max_speed Absolute maximum speed for the motion
         * @param life Life of the motion in ticks, defined as PROCESS_DELAY
         * @param async Whether the motion is asynchronous or not.
         * @param lateral_exit Lateral exit condition
         * @param angular_exit Angular exit condition
         */
        explicit Motion(Pose* pose, float min_speed = 0, float max_speed = 127,
                        uint32_t life = 3000, bool async = false,
                        const ExitCondition &lateral_exit = {1.5, 80}, const ExitCondition &angular_exit = {1.5, 80});

        /**
         * @brief Runs calculations for the motion
         *
         * Is implemented in inheriting motions.
         */
        virtual void update() = 0;

        /**
         * @brief Stores pid controls in pair<float, float> controls variable of base class
         */
        virtual void push_controls() = 0;

        /**
        * @brief Returns pid controls
        */
        virtual std::pair<float, float> get_controls();

        virtual ~Motion() = default;
    };

    class TurnToHeading final : public Motion {
    public:
        float target_heading;

        TurnToHeading(Pose* pose, float target_heading, float min_speed, float max_speed,
            uint32_t life = 3000, bool async = false,
            const ExitCondition &lateral_exit = {MAXFLOAT, 0}, const ExitCondition &angular_exit = {1.5, 80});

        void update() override;

        void push_controls() override;

    private:
        float angular_pid_control = 0;
    };

    class TurnToPoint final : public Motion {
    public:
        Pose target_point;

        TurnToPoint(Pose* pose, const Pose& target_point, float min_speed, float max_speed,
            uint32_t life = 3000, bool async = false,
            const ExitCondition &lateral_exit = {1.5, 80}, const ExitCondition &angular_exit = {1.5, 80});

        void update() override;

        void push_controls() override;

    private:
        float angular_pid_control = 0;
    };

    class MoveToPoint final : public Motion {
    public:
        Pose target_point;

        bool reversed;

        MoveToPoint(Pose* pose, const Pose& target_point, float min_speed, float max_speed,
            uint32_t life = 3000, bool async = false,
            const ExitCondition &lateral_exit = {1.5, 80}, const ExitCondition &angular_exit = {1.5, 80});

        void update() override;

        void push_controls() override;

    private:
        std::pair<float, float> pid_controls {0, 0};

        bool head_stable = false;

        uint32_t head_stable_ticks = 0;

        const uint32_t MinHeadStableTicks = 24;
    };

} // namespace chisel