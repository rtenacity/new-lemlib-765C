#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "main.h"
#include "config.h"


namespace Controller {
    double getForward();
    double getYaw();
    double getLeft();
    double getRight();
    bool getButtonPressed (pros::controller_digital_e_t button);
    bool getDebouncePressed (pros::controller_digital_e_t button);
}

#endif