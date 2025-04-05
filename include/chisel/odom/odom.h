#pragma once

#include "main.h"
#include "chisel/util/util.h"
#include "chisel/chassis/drive.h"
#include "chisel/data/pose.h"
#include "chisel/config.h"
#include <vector>

namespace chisel {

/**
 * @brief Calculates the drift per tick of a Imu.
 *
 * @param imu Reference to the Imu to calculate the drift for.
 * @param timeout Time to calculate the drift for. Longer times generally lead to more accurate results,
 *        though we only have three second at the start of the match. The normal calibration of the Imu takes two,
 *        so best not to set the timeout over 1000 ms
 * @return A Pose with the x and y values being the Imu drift per tick. Its heading should be ignored.
 *
 * @note Most other places timeout/life of processes are denoted by ticks, though its in milliseconds here.
 *       In the future, all timeouts will be in units of milliseconds.
 */
Pose solve_imu_bias(const pros::Imu& imu, uint32_t timeout);

/**
 * @brief Stores the information for a tracking wheel/odometry pod.
 */
struct TrackingWheel {
    pros::Rotation *rotation_sensor;
    Pose offset;
    float wheel_size;

    float prev_pos = 0;

    /**
     * @brief Tracking wheel constructor.
     *
     * @param rotation_sensor Pointer to the rotation sensor of the tracking wheel.
     * @param offset Tracking wheel's location relative to the robot's center.
     *        Heading of offset describes the direction of the tracking wheel.
     * @param wheel_size Diameter of the tracking wheel in inches.
     */
    TrackingWheel(
        pros::Rotation* rotation_sensor, const Pose& offset,
        float wheel_size
    );
};

/**
 * @brief Stores the position tracking information of the robot.
 *
 * In detail, stores the current pose, the Imu and its bias, the drive train, and tracking wheels.
 * Also provides methods to update the position of the robot.
 */
struct Odom {
    Pose pose;
    Pose internal_pose = Pose(0, 0, 0);
    Pose pose_offset;

    Pose ime_estimate = Pose(0, 0, 0);
    Pose odom_estimate = Pose(0, 0, 0);

    Pose imu_bias = Pose(0, 0, 0);

    pros::Imu *imu;
    DriveTrain *drive_train;
    std::vector<TrackingWheel> tracking_wheel_list;

    const int MaxImuResetAttempts;

    float prev_left_pos = 0;
    float prev_right_pos = 0;

    /**
     * @brief Odom constructor.
     *
     * @param pose Reference to the pose of the robot. Can either provide a specific place to update the pose,
     *        or a braced init list if the pose only need be accessed through this struct.
     * @param pose_offset Offset of robot pose; the starting position of the robot.
     *        Internally, the tracking start will still be {0, 0, 0} but output pose will be modified by the offset.
     * \n
     * @param imu Pointer to the Imu used. Set to nullptr if not using an Imu.
     * @param drive_train Pointer to the drive train of the robot.
     * @param tracking_wheel_list_ptr Pointer to the array of tracking wheels. Set to nullptr if not using tracking wheels.
     * @param tracking_wheel_count Number of tracking wheels used. Set to 0 if not using tracking wheels.
     * \n
     * @param MaxImuInitAttempts Maximum number of attempts to calibrate the imu. If the Imu fails after all retries,
     *        position tracking will continue without it. Default is five attempts.
     */
    Odom(
        const Pose &pose,
        const Pose &pose_offset,
        pros::Imu* imu,
        DriveTrain* drive_train,
        TrackingWheel* tracking_wheel_list_ptr,
        int tracking_wheel_count,
        int MaxImuInitAttempts = 5
    );

    /**
     * @brief Initialize the odometry. Will be called by chassis init also, so no need to run twice.
     */
    void initialize();


    /**
     * @brief Calculateds the position using the Imu (if using) and motor encoders.
     *
     * If Imu is nullptr (not using it) heading will still be calculated with the motor encoders.
     * This prediction is stored in the ime_estimate variable.
     */
    void predict_with_ime();

    /**
     * @brief Calculateds the position using the tracking wheels.
     *
     * This prediction is stored in the odom_estimate variable.
     */
    void predict_with_odom();


    /**
     * @brief Updates internal pose with the estimates.
     *
     * Intended to do some sort of filtering to account for inaccuracies of both systems.
     * Filtering isn't implemented yet though, and despite the parameters only Ime estimate will be used.
     * This will change in the future.
     *
     * @param consider_ime Should the method consider the Ime estimate?
     * @param consider_odom Should the method consider the odom estimate?
     */
    void push_prediction(bool consider_ime = true, bool consider_odom = true);

    void load_pose();

private:
    int32_t initialize_imu();
};

} // namespace chisel