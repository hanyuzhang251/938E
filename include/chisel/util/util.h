#pragma once

#include "main.h"
#include "chisel/data/pose.h"
#include <iomanip>
#include <sstream>
#include <cstdint>

#define digi_button controller_digital_e_t
#define anlg_button controller_analog_e_t

#define CTRL_ANGL_LX E_CONTROLLER_ANALOG_LEFT_X
#define CTRL_ANGL_LY E_CONTROLLER_ANALOG_LEFT_Y

#define CTRL_ANGL_RX E_CONTROLLER_ANALOG_RIGHT_X
#define CTRL_ANGL_RY E_CONTROLLER_ANALOG_RIGHT_Y

#define CTRL_DIGI_L1 E_CONTROLLER_DTAL_L1
#define CTRL_DIGI_L2 E_CONTROLLER_DTAL_L2
#define CTRL_DIGI_R1 E_CONTROLLER_DTAL_R1
#define CTRL_DIGI_R2 E_CONTROLLER_DTAL_R2

#define CTRL_DIGI_A E_CONTROLLER_DTAL_A
#define CTRL_DIGI_B E_CONTROLLER_DTAL_B
#define CTRL_DIGI_X E_CONTROLLER_DTAL_X
#define CTRL_DIGI_Y E_CONTROLLER_DTAL_Y

#define CTRL_DIGI_UP E_CONTROLLER_DTAL_UP
#define CTRL_DIGI_DOWN E_CONTROLLER_DTAL_DOWN
#define CTRL_DIGI_LEFT E_CONTROLLER_DTAL_LEFT
#define CTRL_DIGI_RIGHT E_CONTROLLER_DIGITAL_RIGHT

constexpr int INIT_STATE = 0;
constexpr int CRASHOUT = 67;
constexpr int AUTON_STATE = 1;
constexpr int DRIVE_STATE = 2;

/**
 * @brief Analogous to pros::delay()
 *
 * @param delta Time to wait in milliseconds
 */
inline void wait(const uint32_t delta) {
    pros::delay(delta);
}

namespace chisel {
    /**
     * @brief Finds the sign of the given value.
     *
     * @tparam T Type of the given value.
     * @param val given value to find the sign of.
     * @return An int in  the range [-1, 1] describing the sign of the value.
     *
     * @note Will reset in a compile time error if the type of the value does not support required operators.
     */
    template<typename T>
    int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }

    /**
     * @brief Clamps a given value to a given range.
     *
     * @tparam T Type of the given value.
     * @param value Value to clamp.
     * @param min Min value.
     * @param max Max value.
     * @return The clamped value.
     */
    template<typename T>
    T clamp(T value, T min, T max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * @brief Clamps the given value to a given absolute range.
     *
     * If the range was [+20, +50], the value would be clamped to the ranges [-50, -20] and [+20, +50], based on the sign.
     * Because of this, both min and max values are expected to be positive.
     *
     * @tparam T Type of the given value.
     * @param value Value to clamp.
     * @param min Absolute min value.
     * @param max Absolute max value.
     * @return The clamped value.
     */
    template<typename T>
    T abs_clamp(T value, T min, T max) {
        const T abs_val = clamp(std::abs(value), min, max);

        return abs_val * sgn(value);
    }

    /**
     * @brief Normalizes a given degree to the range [-180.0f, +180.0f]
     *
     * @param degree Degree to normalize.
     * @return The normalized degree.
     */
    float deg_norm(float degree);

    /**
     * @brief Finds the error between a current and target degree.
     *        The error is signed so order of parameters must be followed.
     *
     * If the current degree was +45,0f and the target degree was +90,0f, the error would be +45.0f
     * On th eother hand if the target degree of 0.0f, the error would be -45.0f.
     *
     * @param current Current degree.
     * @param target Target degree.
     * @return Degree error, normalized to the range [-180.0f, +180.0f]
     */
    float deg_err(float current, float target);

    /**
     * @brief Finds the degree to a given point with the atan2 function.
     *
     * @param point Reference to the point. Stored in a Pose object though its heading is ignored.
     * @return The degree to the point.
     */
    float deg_to_point(const Pose &point);

    /**
     * @brief Finds the distance to a give point using the Pythagreon theorom.
     *
     * The point is assumed to be relative, as in the function will not account for current robot position.
     *
     * @param point
     * @return
     */
    float dist_to_point(const Pose &point);

    /**
     * @brief Formates a provided millisecond value to mm:ss:___
     *        TODO find out what the millisecond sign thing is
     *
     * @param milliseconds Milliseconds to format.
     * @return Formated string as a std::string.
     */
    std::string format_millis(uint32_t milliseconds);

    /**
     * @brief Timestamp prefix for logs.
     *
     * @return An std::string of length 15, containing the current timstamp in the format "[mm:ss:___]:   "
     */
    std::string prefix();

    /**
     * @brief Starterpack for a toggle button using true/false inputs from controller.
     *
     * Controller provides true/false based on if buttons are pressed.
     * Making toggle buttons with this system would require two additional variables for each button,
     * additionally taking up two additional names. Instead, package everything into a struct.
     *
     * @note This system can also easily be used to make on press buttons by setting the value back to false every time it is set to true.
     */
    struct Toggle {
        bool value;
        bool ptrigger;

        /**
         * @brief Toggle constructor
         *
         * @param value_ Starting value of the toggle.
         */
        explicit Toggle(bool value_ = false);

        /**
         * @brief Updates the toggle with whether the button is pressed or not.
         *
         * @param trigger Is the button pressed?
         */
        void tick(bool trigger);
    };
} // namespace chisel
