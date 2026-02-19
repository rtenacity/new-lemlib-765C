#include "main.h"
#include "routes.h"
#include "controller.h"
#include "chassis.h"
#include "effectors.h"
#include "mcl.h"

// Create robodash autonomous selector
rd::Selector selector({

	{"Left Side", Routes::leftSide},
	{"Right Side", Routes::rightSide},
	{"Skills Auton", Routes::skillsAuton},
	{"Stupid Skills", Routes::stupidSkillsAuton}

});

// Create robodash console
rd::Console console;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	// Selector callback function, prints selected auton to the console
	selector.on_select([](std::optional<rd::Selector::routine_t> routine)
					   {
		if (routine == std::nullopt) {
			std::cout << "No routine selected" << std::endl;
		} else {
			std::cout << "Selected Routine: " << routine.value().name << std::endl;
		} });

	Chassis::init(); // initialize chassis

	// Routes::skillsAuton();

	// Effectors::toggleMatchLoader();

	// Effectors::toggleMiddlePiston();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
	selector.focus();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{

	selector.run_auton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

	int i = 0;
	int driveReversed = 1;
	int yawFactor = 1; // Tune this based on your driver's preference

	Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	Chassis::getChassis().setPose(0, 0, 0);

	// ################################ SKILLS ################################ //

	Effectors::toggleLowerStage();

	Chassis::getChassis().moveToPose(-15, 24, -46, 3000, {.lead = 0.6, .maxSpeed = 40, .earlyExitRange = 2}, true);

	pros::delay(1600);

	Effectors::toggleMatchLoader();

	Chassis::getChassis().waitUntilDone();

	pros::delay(600);

	Effectors::toggleMatchLoader();

	Effectors::toggleLowerStage();

	Chassis::getChassis().turnToPoint(1, 36, 1000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 5}, false);

	Chassis::getChassis().moveToPose(1, 35, -130, 100000, {.forwards = false, .lead = 0.2, .earlyExitRange = 3}, false);

	Effectors::updateIntakeSpeed(470);

	Effectors::toggleIntakeDirection();

	Effectors::toggleLowerStage();

	pros::delay(100);

	Effectors::toggleLowerStage();

	Effectors::toggleIntakeDirection();

	Effectors::toggleMiddlePiston();

	Effectors::toggleLowerStage();

	Effectors::toggleUpperStage();

	pros::delay(2000);

	Effectors::toggleUpperStage();

	Effectors::toggleLowerStage();

	Effectors::updateIntakeSpeed(600);

	Effectors::toggleMiddlePiston();

	Chassis::getChassis().moveToPoint(-36, 0, 100000, {.maxSpeed = 100, .earlyExitRange = 1}, false);

	Chassis::getChassis().turnToHeading(-180, 100000, {.earlyExitRange = 2}, false);

	Effectors::toggleMatchLoader();

	Effectors::toggleLowerStage();

	Chassis::getChassis().moveToPoint(-36.5, -14.5, 2700, {.earlyExitRange = 1}, false);

	pros::delay(1500);

	Effectors::toggleLowerStage();

	Chassis::getChassis().moveToPoint(-37, 13, 3000, {.forwards = false, .maxSpeed = 100, .earlyExitRange = 1}, false);

	Effectors::toggleUpperStage();

	Effectors::toggleLowerStage();

	pros::delay(2000);

	Effectors::toggleMatchLoader();

	Effectors::toggleUpperStage();

	Effectors::toggleLowerStage();

	Chassis::getChassis().moveToPoint(-37, 0, 3000, {.earlyExitRange = 1}, false);

	//! Untested as of now!!

	Chassis::getChassis().moveToPose(-8, -18, -270, 6000, {.forwards = false, .lead = 0.4, .maxSpeed = 127, .earlyExitRange = 2}, false);

	Effectors::toggleMatchLoader();

	pros::delay(500);

	Chassis::getChassis().tank(127, 127);

	pros::delay(950);

	Effectors::toggleLowerStage();

	Chassis::getChassis().tank(0, 0);

	Effectors::toggleMatchLoader();

	// ################################ RIGHT SIDE AUTON ################################ //

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().moveToPoint(-10, 25, 100000, {.maxSpeed = 40, .earlyExitRange = 2}, true);

	// pros::delay(1500);

	// Effectors::toggleMatchLoader();

	// Chassis::getChassis().waitUntilDone();

	// pros::delay(600);

	// Effectors::toggleMatchLoader();

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().turnToHeading(-130, 1500, {.earlyExitRange = 2}, false);

	// Chassis::getChassis().moveToPoint(-35.7, 0, 100000, {.maxSpeed = 100,}, false);

	// Chassis::getChassis().turnToHeading(180, 1500, {.earlyExitRange = 2}, false);

	// Chassis::getChassis().moveToPose(-36.2, 16, 180, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 1}, false);

	// Effectors::toggleLowerStage();

	// Effectors::toggleUpperStage();

	// pros::delay(2000);

	// Effectors::toggleUpperStage();

	// Effectors::toggleMatchLoader();

	// Chassis::getChassis().moveToPoint(-35, -13.5, 3000, {.earlyExitRange = 2}, false);

	// pros::delay(500);

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().moveToPoint(-36.5, 12, 2000, {.forwards = false, .maxSpeed = 100, .earlyExitRange = 2}, false);

	// Effectors::toggleLowerStage();

	// Effectors::toggleUpperStage();

	// ################################ LEFT SIDE AUTON ################################ //

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().moveToPoint(10, 25, 100000, {.maxSpeed = 40, .earlyExitRange = 2}, true);

	// pros::delay(1500);

	// Effectors::toggleMatchLoader();

	// Chassis::getChassis().waitUntilDone();

	// pros::delay(600);

	// Effectors::toggleMatchLoader();

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().turnToHeading(130, 1500, {.earlyExitRange = 2}, false);

	// Chassis::getChassis().moveToPoint(35, 0, 100000, {.maxSpeed = 100,}, false);

	// Chassis::getChassis().turnToHeading(180, 1500, {.earlyExitRange = 2}, false);

	// Chassis::getChassis().moveToPose(35.5, 15, 180, 100000, {.forwards = false, .maxSpeed = 127, .earlyExitRange = 1}, false);

	// Effectors::toggleLowerStage();

	// Effectors::toggleUpperStage();

	// pros::delay(2000);

	// Effectors::toggleUpperStage();

	// Effectors::toggleMatchLoader();

	// Chassis::getChassis().moveToPoint(34.5, -15, 100000, {.earlyExitRange = 2}, false);

	// pros::delay(600);

	// Effectors::toggleLowerStage();

	// Chassis::getChassis().moveToPoint(35, 12, 2000, {.forwards = false, .maxSpeed = 100, .earlyExitRange = 2}, false);

	// Effectors::toggleLowerStage();

	// Effectors::toggleUpperStage();

	// move to the middle goal and score

	// Chassis::getChassis().turnToHeading(-135+180, 1500, {.maxSpeed = 80, .earlyExitRange = 2}, true);

	// pros::delay(700);

	// Chassis::getChassis().moveToPose(4, 33, -135+180, 3000, {.forwards = false, .maxSpeed = 80, .earlyExitRange = 2}, true);	// NEEDS TO BE TUNED

	// Effectors::toggleMiddlePiston();

	// Chassis::getChassis().waitUntilDone();

	// Effectors::toggleUpperStage();

	// pros::delay(3000);

	// // move to the matchloader

	// Chassis::getChassis().moveToPoint(34, 14.5, 100000, {.maxSpeed = 120, .earlyExitRange = 2}, true); // in front of matchloader

	// Chassis::getChassis().turnToHeading(180, 100000, {.maxSpeed = 80, .earlyExitRange = 2}, true);

	// Chassis::getChassis().moveToPoint(40, 3, 100000, {.maxSpeed = 80, .earlyExitRange = 2}, true); // matchloader

	// Chassis::getChassis().moveToPoint(-30, 27, 100000, {.maxSpeed = 80, .earlyExitRange = 2}, true); score

	/*



	*/

	while (true)
	{

		// if (i % 100 == 0) {
		// 	MCL::getSensorValues();
		// }
		double forward = Controller::getForward();
		double yaw = Controller::getYaw();

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_L2))
		{
			driveReversed = -driveReversed;
		}

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_L1))
		{
			Effectors::toggleIntakeDirection();
		}

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_B))
		{
			Effectors::toggleMatchLoader();
		}

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_R1))
		{
			Effectors::toggleLowerStage();
		}

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_R2))
		{
			Effectors::toggleUpperStage();
		}

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_X))
		{
			Effectors::toggleMiddlePiston();
		}

		Chassis::getChassis().arcade(forward * -driveReversed, yaw * yawFactor);

		i++;

		pros::delay(5);
	}
}