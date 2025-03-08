#pragma once

#include "main.h"
#include "version.h"
#include "chassis.h"
#include "util.h"
#include "pid.h"
#include "drive.h"
#include "odom.h"

namespace chisel {
    
Chassis chassis;

void setChassis(Chassis newChassis) {
    chassis = newChassis;
}

}