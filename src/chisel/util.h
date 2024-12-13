#ifndef UTIL_H
#define UTIL_H

#include "main.h"
#include <atomic>

namespace chisel {
    #define digi controller_digital_e_t
    #define anlg controller_analog_e_t

    #define CTRL_ANLG_LX E_CONTROLLER_ANALOG_LEFT_X
    #define CTRL_ANLG_LY E_CONTROLLER_ANALOG_LEFT_Y

    #define CTRL_ANLG_RX E_CONTROLLER_ANALOG_RIGHT_X
    #define CTRL_ANLG_RY E_CONTROLLER_ANALOG_RIGHT_Y

    #define CTRL_DIGI_L1 E_CONTROLLER_DIGITAL_L1
    #define CTRL_DIGI_L2 E_CONTROLLER_DIGITAL_L2
    #define CTRL_DIGI_R1 E_CONTROLLER_DIGITAL_R1
    #define CTRL_DIGI_R2 E_CONTROLLER_DIGITAL_R2

    #define CTRL_DIGI_A E_CONTROLLER_DIGITAL_A
    #define CTRL_DIGI_B E_CONTROLLER_DIGITAL_B
    #define CTRL_DIGI_X E_CONTROLLER_DIGITAL_X
    #define CTRL_DIGI_Y E_CONTROLLER_DIGITAL_Y

    #define CTRL_DIGI_UP E_CONTROLLER_DIGITAL_UP
    #define CTRL_DIGI_DOWN E_CONTROLLER_DIGITAL_DOWN
    #define CTRL_DIGI_LEFT E_CONTROLLER_DIGITAL_LEFT
    #define CTRL_DIGI_RIGHT E_CONTROLLER_DIGITAL_RIGHT

    typedef std::atomic<float> at_f;

    struct pose {
        float x_pos;
        float y_pos;
        float heading;
    };

    template <typename T> int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }
}

#endif //UTIL_H