#include "routes.h"

namespace Routes
{
    void leftSide()
    {

        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(0, 0, 0);

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPoint(0, 10, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

        Chassis::getChassis().moveToPose(-8, 22, -38, 100000, {.maxSpeed = 100, .earlyExitRange = 5}, false);

        pros::delay(1000);

        Chassis::getChassis().turnToPoint(-36, -1.5, 1200, {.maxSpeed = 127, .earlyExitRange = 3}, true);

        pros::delay(800);

        Effectors::toggleLowerStage();

        Chassis::getChassis().waitUntilDone();

        Chassis::getChassis().moveToPoint(-36, -1.5, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().turnToHeading(179, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().moveToPoint(-36, 17, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 2}, false);

        Effectors::toggleIntakeDirection();

        Effectors::toggleLowerStage();

        pros::delay(300);

        Effectors::toggleUpperStage();

        Effectors::toggleIntakeDirection();


        pros::delay(2000);

        Effectors::toggleUpperStage();

        Effectors::toggleMatchLoader();

        Chassis::getChassis().moveToPoint(-37, -14, 100000, {.forwards = true, .maxSpeed = 127, .earlyExitRange = 2}, false);

        pros::delay(800);

        Chassis::getChassis().moveToPoint(-37, 17, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 2}, false);

        Effectors::toggleUpperStage();



    }

    void rightSide()
    {

        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(0, 0, 0);
        Effectors::toggleLowerStage();
        Chassis::getChassis().moveToPoint(0, 10, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

        Chassis::getChassis().moveToPose(8, 22, 38, 100000, {.maxSpeed = 100, .earlyExitRange = 5}, false);

        Chassis::getChassis().turnToPoint(34, -1, 1200, {.maxSpeed = 127, .earlyExitRange = 3}, true);

        pros::delay(600);

        Effectors::toggleLowerStage();

        Chassis::getChassis().waitUntilDone();

        Chassis::getChassis().moveToPoint(34, -1, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().turnToHeading(179, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().moveToPoint(34, 16, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 2}, false);

        Effectors::toggleIntakeDirection();

        Effectors::toggleLowerStage();

        pros::delay(250);

        Effectors::toggleIntakeDirection();

        Effectors::toggleUpperStage();
    }

    void skillsAuton()
    {

        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(-46.8, 10.421, 90);

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPose(-19.948, 30.447, 25, 100000, {.earlyExitRange = 3}, true); //! Tune lead

        pros::delay(900); //! Tune delay

        Effectors::toggleMatchLoader();


        pros::delay(300);


        Effectors::toggleMatchLoader();

        Chassis::getChassis().waitUntilDone();

        Chassis::getChassis().moveToPoint(-24.42, 24.031, 100000, {.forwards=false}, false);

        Effectors::toggleLowerStage();

        Chassis::getChassis().turnToPoint(-9.644, 10.421, 100000, {.earlyExitRange=3}, false);

        Chassis::getChassis().moveToPoint(-9.644, 10.421, 100000, {.earlyExitRange=3}, false);

        Effectors::toggleMiddlePiston();

        Effectors::toggleLowerStage();
        Effectors::toggleUpperStage();

        pros::delay(1000);

// chassis.moveTo(-19.948, 30.447, 5000);
// chassis.moveTo(-24.42, 24.031, 5000);
// chassis.moveTo(-9.644, 10.421, 5000);


// chassis.moveTo(-46.973, 46.39, 5000);
// chassis.moveTo(-62.333, 46.39, 5000);
// chassis.moveTo(-27.142, 46.585, 5000);
// chassis.moveTo(-46.585, 46.39, 5000);
// chassis.moveTo(-46.585, -47.518, 5000);
// chassis.moveTo(-63.5, -46.935, 5000);
// chassis.moveTo(-26.559, -47.518, 5000);
// chassis.moveTo(-63.305, -19.52, 5000);











       
    }

    void stupidSkillsAuton()
    {
        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(0, 0, 0);

        Chassis::getChassis().moveToPoint(0, -4, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

        Effectors::toggleMatchLoader();

        pros::delay(500);

        Chassis::getChassis().tank(127, 127);

        pros::delay(950);

        Effectors::toggleLowerStage();

        Chassis::getChassis().tank(0, 0);

        Effectors::toggleMatchLoader();
    }
}