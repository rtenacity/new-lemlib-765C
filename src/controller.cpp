#include "controller.h"

namespace Controller {

    pros::Controller controller (pros::E_CONTROLLER_MASTER);

    double getForward() {
        return controller.get_analog(FORWARD_CONTROL);
    }

    double getYaw() {
        return controller.get_analog(YAW_CONTROL);
    }

    double getLeft() {
        return controller.get_analog(LEFT_CONTROL);
    }

    double getRight() {
        return controller.get_analog(RIGHT_CONTROL);
    }

    bool getButtonPressed (pros::controller_digital_e_t button) {
        return controller.get_digital(button);
    }

    bool getDebouncePressed (pros::controller_digital_e_t button) {
        return controller.get_digital_new_press(button); 
    }

}