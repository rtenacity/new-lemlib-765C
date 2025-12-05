#include "main.h"
#include "routes.h"
#include "controller.h"
#include "chassis.h"
#include "effectors.h"

// Create robodash autonomous selector
rd::Selector selector({

	{"Left Side", Routes::leftSide},
	{"Right Side", Routes::rightSide},
	{"Skills Auton", Routes::skillsAuton},

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







	Chassis::getChassis().setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	while (true)
	{
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

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_Y))
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

		if (Controller::getDebouncePressed(pros::E_CONTROLLER_DIGITAL_B))
		{
			Effectors::toggleMiddlePiston();
		}


		Chassis::getChassis().arcade(forward * driveReversed, yaw * yawFactor);

		i++;

		pros::delay(5);
	}
}