#include "routes.h"

namespace Routes
{
    void leftSide()
    {

        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        pros::delay(50);

        Chassis::getChassis().setPose(0, 0, 0);

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPoint(-0, 18, 1000000, {.maxSpeed = 127, .earlyExitRange = 5}, false); // motion chain

        Chassis::getChassis().moveToPose(-7, 32, -34, 2800, {.maxSpeed = 127, .earlyExitRange = 5}, false); // motion chain

        pros::delay(1000);

        Chassis::getChassis().turnToPoint(-32, 8, 1000000, {}, false); // motion chain

        Chassis::getChassis().moveToPoint(-32, 8, 1000000, {}, false); // motion chain

        Chassis::getChassis().turnToHeading(-180, 1000, {.earlyExitRange = 2}, false); // motion chain


        Chassis::getChassis().moveToPoint(-33, 28, 2000,  {.forwards = false, .maxSpeed=80}, true); // motion chain

        Effectors::toggleIntakeDirection();


        pros::delay(350);

        Effectors::toggleIntakeDirection();

        Chassis::getChassis().waitUntilDone();

        Effectors::toggleUpperStage();
    }

    void rightSide()
    {

       Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        pros::delay(50);

        Chassis::getChassis().setPose(0, 0, 0);

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPoint(-0, 18, 1000000, {.maxSpeed = 127, .earlyExitRange = 5}, false); // motion chain

        Chassis::getChassis().moveToPose(7, 32, -34, 2800, {.maxSpeed = 127, .earlyExitRange = 5}, false); // motion chain

        pros::delay(1000);

        Chassis::getChassis().turnToPoint(32, 8, 1000000, {}, false); // motion chain

        Chassis::getChassis().moveToPoint(32, 8, 1000000, {}, false); // motion chain

        Chassis::getChassis().turnToHeading(-180, 1000, {.earlyExitRange = 2}, false); // motion chain


        Chassis::getChassis().moveToPoint(33, 28, 2000,  {.forwards = false, .maxSpeed=80}, true); // motion chain

        Effectors::toggleIntakeDirection();


        pros::delay(350);

        Effectors::toggleIntakeDirection();

        Chassis::getChassis().waitUntilDone();

        Effectors::toggleUpperStage();
    }

   

    void skillsAuton()
    {

        std::cout << "Skills Auton" << std::endl;
        Chassis::getChassis().moveToPoint(0, 24, 1000000, {}, false);
    }
}