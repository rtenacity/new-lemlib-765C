#include "effectors.h"

namespace Effectors
{
    pros::ADIDigitalOut piston(SAMPLE_PISTON);
    pros::Motor bottomIntake(FIRST_STAGE, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::Motor topIntake(SECOND_STAGE, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

    bool intakeState = false;
    bool toggleLowerStageState = false;

    int intakeDirection = -1;
    int upperSpeed = 600;
    bool pistonState = false;

    void togglePiston()
    {
        pistonState = !pistonState;
        piston.set_value(pistonState);
    }

    void toggleIntakeDirection() {
        intakeDirection = -intakeDirection;
        if (intakeState) {
            topIntake.move_velocity(intakeDirection * upperSpeed);
        }
        if (toggleLowerStageState) {
            bottomIntake.move_velocity(intakeDirection * 600);
        }
    }

    void toggleLowerStage() {
        toggleLowerStageState = !toggleLowerStageState;
        if (toggleLowerStageState) {
            bottomIntake.move_velocity(intakeDirection * 600);
        } else {
            bottomIntake.move_velocity(0);
        }
    }   

    void toggleUpperStage() {
        intakeState = !intakeState;
        if (intakeState) {
            topIntake.move_velocity(intakeDirection * upperSpeed);
        } else {
            topIntake.move_velocity(0);
        }
    }

    void toggleUpperStageSpeed() {
        if (upperSpeed == intakeDirection * 600) {
            upperSpeed = 50;
            std::cout << "Upper stage speed set to -300" << std::endl;
        } else {
            upperSpeed = intakeDirection * 600;
        }
        if (intakeState) {
            topIntake.move_velocity(upperSpeed);
        }
    }

    
}