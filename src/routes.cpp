#include "routes.h"

namespace Routes
{
	void leftSide()
	{

		Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(-12, 25, 100000, {.maxSpeed = 40, .earlyExitRange = 2}, true);

		pros::delay(1500);

		Effectors::toggleMatchLoader();

		Chassis::getChassis().waitUntilDone();

		pros::delay(600);

		Effectors::toggleMatchLoader();

		Effectors::toggleLowerStage();

		Chassis::getChassis().turnToHeading(-130, 1000, {.earlyExitRange = 2}, false);

		Chassis::getChassis().moveToPoint(-37, 0, 2000, {
															.maxSpeed = 100,
														},
										  false);

		Chassis::getChassis().turnToHeading(182, 1500, {.earlyExitRange = 2}, false);

		Chassis::getChassis().moveToPose(-37, 16, 180, 2500, {.forwards = false, .earlyExitRange = 1}, false);

		Effectors::toggleLowerStage();

		Effectors::toggleUpperStage();

		pros::delay(1500);

		Effectors::toggleUpperStage();

		Effectors::toggleMatchLoader();

		Chassis::getChassis().moveToPoint(-37, -15, 3000, {.earlyExitRange = 3}, false);

		pros::delay(600);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(-37, 15, 2000, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 2}, false);

		Effectors::toggleLowerStage();

		Effectors::toggleUpperStage();
	}

	void rightSide()
	{

		Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(12, 25, 100000, {.maxSpeed = 40, .earlyExitRange = 2}, true);

		pros::delay(1500);

		Effectors::toggleMatchLoader();

		Chassis::getChassis().waitUntilDone();

		pros::delay(600);

		Effectors::toggleMatchLoader();

		Effectors::toggleLowerStage();

		Chassis::getChassis().turnToHeading(130, 1500, {.earlyExitRange = 2}, false);

		Chassis::getChassis().moveToPoint(34.5, 0, 100000, {
															 .maxSpeed = 100,
														 },
										  false);

		Chassis::getChassis().turnToHeading(180, 1500, {.earlyExitRange = 2}, false);

		Chassis::getChassis().moveToPose(34.5, 16, 180, 2500, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 1}, false);

		Effectors::toggleLowerStage();

		Effectors::toggleUpperStage();

		pros::delay(1500);

		Effectors::toggleUpperStage();

		Effectors::toggleMatchLoader();

		Chassis::getChassis().moveToPoint(34.5, -16, 3000, {.earlyExitRange = 2}, false);

		pros::delay(600);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(34, 15, 2000, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 2}, false);

		Effectors::toggleLowerStage();

		Effectors::toggleUpperStage();
	}

	void skillsAuton()
	{

		Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

		Chassis::getChassis().setPose(0, 0, 0);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPose(-15, 24, -46, 3000, {.lead = 0.6, .maxSpeed = 40, .earlyExitRange = 2}, true);

		pros::delay(1700);

		Effectors::toggleMatchLoader();

		Chassis::getChassis().waitUntilDone();

		pros::delay(600);

		Effectors::toggleMatchLoader();

		Effectors::toggleLowerStage();

		pros::delay(300);

		Chassis::getChassis().moveToPoint(-37, 0, 100000, {.maxSpeed = 100, .earlyExitRange = 1}, false);

		Chassis::getChassis().turnToHeading(-180, 750, {.earlyExitRange = 3}, false);

		Chassis::getChassis().moveToPoint(-37, 15, 3000, {.forwards = false, .maxSpeed = 100, .earlyExitRange = 1}, false);

		Effectors::toggleUpperStage();

		Effectors::toggleLowerStage();

		pros::delay(300);

		pros::delay(2000);

		Effectors::toggleUpperStage();

		Effectors::toggleLowerStage();

		Effectors::toggleMatchLoader();

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(-36.5, -18, 2700, {.maxSpeed = 80, .earlyExitRange = 1}, false);

		pros::delay(1500);

		Effectors::toggleLowerStage();

		Chassis::getChassis().moveToPoint(-37, 14, 3000, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 1}, false);

		Effectors::toggleIntakeDirection();

		Effectors::toggleUpperStage();

		Effectors::toggleLowerStage();

		pros::delay(200);

		Effectors::toggleIntakeDirection();

		pros::delay(3000);

		Effectors::toggleUpperStage();

		Effectors::toggleLowerStage();

		Effectors::toggleMatchLoader();

		Chassis::getChassis().moveToPoint(-37.5, -4, 3000, {.earlyExitRange = 1}, false);

		Chassis::getChassis().turnToHeading(90, 1500, {.earlyExitRange = 2}, false);

		Chassis::getChassis().moveToPose(-6, -25, 90, 5000, {.lead = 0.4, .maxSpeed = 127, .earlyExitRange = 1}, false);

		Chassis::getChassis().turnToHeading(-275, 1500, {.earlyExitRange = 1}, false);

		Effectors::toggleMatchLoader();

		pros::delay(500);

		Chassis::getChassis().tank(127, 127);

		pros::delay(500);

		Effectors::toggleLowerStage();
		Effectors::toggleUpperStage();

		Effectors::toggleMatchLoader();

		pros::delay(350);

		Chassis::getChassis().tank(0, 0);

		pros::delay(500);
	}

	void stupidSkillsAuton()
	{
		Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

		Chassis::getChassis().setPose(0, 0, 0);

		Chassis::getChassis().moveToPoint(0, -4, 100000, {.maxSpeed = 127, .earlyExitRange = 3}, false);

		Effectors::toggleMatchLoader();

		pros::delay(500);

		Chassis::getChassis().tank(127, 127);

		pros::delay(300);

		Effectors::toggleMatchLoader();

		Effectors::toggleLowerStage();

		pros::delay(550);

		Chassis::getChassis().tank(0, 0);

		// Effectors::toggleMatchLoader();
	}
}