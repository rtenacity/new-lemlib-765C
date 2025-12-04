#include "effectors.h"

namespace Effectors
{
    pros::ADIDigitalOut matchLoader(SAMPLE_PISTON); // match loader piston
    pros::ADIDigitalOut middlePiston(MIDDLE_PISTON); // middle goal piston
    pros::Motor bottomIntake(FIRST_STAGE, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
    pros::Motor topIntake(SECOND_STAGE, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

    bool toggleUpperStageState = false; // upper stage state
    bool toggleLowerStageState = false; // lower stage state
    bool middlePistonState = false;    // middle piston state

    int intakeDirection = -1; // intake direction, -1 is default (intake in)
    int upperSpeed = 0; // default zero
    int lowerSpeed = 0; // default zero

    bool matchLoaderState = false;

    void toggleMatchLoader()
    {
        matchLoaderState = !matchLoaderState;
        matchLoader.set_value(matchLoaderState);
    }

    void toggleMiddlePiston()
    {
        middlePistonState = !middlePistonState;
        middlePiston.set_value(middlePistonState);
    }



    void toggleIntakeDirection()
    {
        // reverse intake direction
        intakeDirection = -intakeDirection;

        if (toggleLowerStageState)
        {
            // lower is on, set to new direction
            bottomIntake.move_velocity(intakeDirection * 600);
        }
        else
        {
            // lower is off
            bottomIntake.move_velocity(0);
        }

        if (toggleUpperStageState) // upper stage
        {
            if (toggleLowerStageState)
            {
                // lower is on, set to new direction
                topIntake.move_velocity(intakeDirection * 600);
            }
            else
            {
                // lower is off, set to new direction
                topIntake.move_velocity(intakeDirection * 600);
            }
        }
        else
        {
            // upper is off
            if (toggleLowerStageState) {
                // Maintain indexing behavior even when reversing
                topIntake.move_velocity(-20 * intakeDirection); 
            } else {
                topIntake.move_velocity(0);
            }
        }
    }

    void toggleLowerStage()
    {
        toggleLowerStageState = !toggleLowerStageState;

        if (toggleLowerStageState)
        {
            // lower on
            bottomIntake.move_velocity(intakeDirection * 600);

            // if upper is on while lower turns on
            if (toggleUpperStageState)
            {
                topIntake.move_velocity(intakeDirection * 600);
            }
            else
            {
                // lower is on, upper is off == set to -20
                topIntake.move_velocity(-20 * intakeDirection);
            }
        }
        else
        {
            // lower turns off
            bottomIntake.move_velocity(0);

            // upper must also turn off
            toggleUpperStageState = false;
            topIntake.move_velocity(0);
        }
    }

    void toggleUpperStage()
    {
        // if lower is off, upper toggles normally
        // if lower is on, upper has special off speed

        toggleUpperStageState = !toggleUpperStageState;

        if (toggleUpperStageState)
        {
            // toggled on
            topIntake.move_velocity(intakeDirection * 600);
        }
        else
        {
            // toggled off
            if (toggleLowerStageState)
            {
                // lower is on, use special fallback speed
                topIntake.move_velocity(-20 * intakeDirection);
            }
            else
            {
                // lower is off, upper goes to zero
                topIntake.move_velocity(0);
            }
        }
    }

}
