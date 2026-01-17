#include "chassis.h"

namespace Chassis
{
    pros::MotorGroup left_motors({-LEFT_BOT_MOTOR, -LEFT_MID_MOTOR, -LEFT_TOP_MOTOR}, pros::MotorGearset::blue);
    pros::MotorGroup right_motors({RIGHT_BOT_MOTOR, RIGHT_MID_MOTOR, RIGHT_TOP_MOTOR}, pros::MotorGearset::blue);

    lemlib::Drivetrain drivetrain(&left_motors,               // left motor group
                                  &right_motors,              // right motor group
                                  13,                         // 13 inch track width
                                  lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                                  480,                        // drivetrain rpm is 480
                                  2                           // horizontal drift is 2 (for now)
    );

    pros::Rotation trackingWheel(-TRACKING_PORT); // rotation sensor, reversed for proper orientation
    lemlib::TrackingWheel vertical_tracking_wheel(&trackingWheel, lemlib::Omniwheel::NEW_275_HALF, 0); // 2.75 inch tracking wheel
    pros::Imu imu(IMU_PORT);

    lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                                nullptr,                  // vertical tracking wheel 2, set to nullptr
                                nullptr,                  // horizontal tracking wheel 1, set to nullptr
                                nullptr,                  // horizontal tracking wheel 2, set to nullptr
                                &imu                      // inertial sensor
    );

    lemlib::ControllerSettings lateral_controller(5.1,  // proportional gain (kP)
                                                  0,    // integral gain (kI)
                                                  1.9,  // derivative gain (kD)
                                                  3,    // anti windup
                                                  1, // small error range, in inches
                                                  100,  // small error range timeout, in milliseconds
                                                  2,    // large error range, in inches
                                                  500,  // large error range timeout, in milliseconds
                                                  3    // maximum acceleration (slew)
    );

    // angular PID controller
    lemlib::ControllerSettings angular_controller(1.55,   // proportional gain (kP)
                                                  0, // integral gain (kI)
                                                  4.3,  // derivative gain (kD)
                                                  3,   // anti windup
                                                  2, // small error range, in degrees
                                                  100, // small error range timeout, in milliseconds
                                                  3,   // large error range, in degrees
                                                  500, // large error range timeout, in milliseconds
                                                  100   // maximum acceleration (slew)
    );

    lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                            lateral_controller, // lateral PID settings
                            angular_controller, // angular PID settings
                            sensors             // odometry sensors
    );

    void init()
    {
        chassis.calibrate();
        chassis.getPose();

        pros::Task screen_task([&]()
                               {
            int i = 0;
            while (true) {
                if (i % 30 == 0) {
                    std::cout << "X: " << chassis.getPose().x << " Y: " << chassis.getPose().y << " Theta: " << static_cast<std::int32_t>(chassis.getPose().theta) % 360 << std::endl;
                } 

                i++;  

                pros::delay(5);
            } });
    }

    lemlib::Chassis &getChassis()
    {
        return chassis;
    }

}