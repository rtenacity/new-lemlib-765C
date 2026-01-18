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

        Chassis::getChassis().turnToPoint(-34, -1.5, 1200, {.maxSpeed = 127, .earlyExitRange = 3}, true);

        pros::delay(800);

        Effectors::toggleLowerStage();

        Chassis::getChassis().waitUntilDone();

        Chassis::getChassis().moveToPoint(-34.5, -1.5, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().turnToHeading(179, 100000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().moveToPoint(-34.5, 17, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 2}, false);

        Effectors::toggleIntakeDirection();

        Effectors::toggleLowerStage();

        pros::delay(250);

        Effectors::toggleUpperStage();

        Effectors::toggleIntakeDirection();

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

        // MOVE TO BLOCKS AND INTAKE

        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(0, 0, 0);

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPoint(0, 10, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

        Chassis::getChassis().moveToPose(11, 21, 44, 2000, {.maxSpeed = 100, .earlyExitRange = 5}, false);

        Effectors::toggleMatchLoader();     // this "catches" any blocks that would roll out otherwise

        pros::delay(1000);

        Chassis::getChassis().moveToPose(11.5, 24, 90, 2000, {.maxSpeed = 100, .earlyExitRange = 5}, false);    // adjust from 45 to 60

        // pros::delay(1000);

        Effectors::toggleLowerStage();



        // MOVE TO LONG GOAL

        Chassis::getChassis().turnToPoint(33.5, -1, 800, {.maxSpeed = 127, .earlyExitRange = 5}, false);

        Chassis::getChassis().moveToPoint(33.5, -1, 2000, {.maxSpeed = 127, .earlyExitRange = 5}, false);

        Chassis::getChassis().turnToHeading(179, 2000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        Chassis::getChassis().moveToPoint(33.5, 17, 20000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 5}, false);

        Effectors::toggleLowerStage();

        Effectors::toggleUpperStage();

        pros::delay(3000);

        // MOVE TO MATCHLOADER

        Effectors::toggleLowerStage();

        Chassis::getChassis().moveToPose(34, -13.5, 180, 10000, {.maxSpeed = 127, .earlyExitRange = 2}, false);

        pros::delay(500);

        Effectors::toggleLowerStage();

        // Effectors::toggleMatchLoader();

        // Effectors::toggleLowerStage();

        // pros::delay(1000);

        // Chassis::getChassis().moveToPoint(34, -11, 100000, {.maxSpeed = 127, .earlyExitRange = 5}, false);


        // move to southeast matchloader

        // Chassis::getChassis().tank(127, 127);

        // pros::delay(700);

        // Chassis::getChassis().tank(0, 0);

        // int oscillation_num = 4;

        // for (int i = 0; i < oscillation_num; i++)
        // {

        //     Chassis::getChassis().tank(30, 30);

        //     pros::delay(200);

        //     Chassis::getChassis().tank(-30, -30);

        //     pros::delay(150);

        //     Chassis::getChassis().tank(0, 0);
        // }

    //     Effectors::toggleLowerStage();

    //     Chassis::getChassis().moveToPoint(34.2, 10, 100000, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 5}, false);

    //     Effectors::toggleIntakeDirection();

    //     Effectors::toggleLowerStage();

    //     pros::delay(250);

    //     Effectors::toggleIntakeDirection();

    //     Effectors::toggleUpperStage();

    //     pros::delay(1200);

    //     Effectors::toggleLowerStage();

    //     Effectors::toggleUpperStage();

    //     Effectors::toggleMatchLoader();

    //     Chassis::getChassis().moveToPose(8, -18, 270, 5000, {.maxSpeed = 127, .earlyExitRange = 5}, false);

    //     Chassis::getChassis().moveToPoint(14, -18, 5000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 5}, false);
    }

    void stupidSkillsAuton()
    {
        Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

        Chassis::getChassis().setPose(0, 0, 0);

        Chassis::getChassis().moveToPoint(0, -6, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

        Effectors::toggleMatchLoader();

        pros::delay(500);

        Chassis::getChassis().tank(127, 127);

        pros::delay(750);

        Effectors::toggleLowerStage();

        Chassis::getChassis().tank(0, 0);

        Effectors::toggleMatchLoader();
    }
}