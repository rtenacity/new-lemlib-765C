// #include "arm.h"

// namespace Arm
// {
//     pros::Motor armMotor(ARM_MOTOR, pros::MotorGearset::blue);
//     pros::Rotation armSensor(ARM_SENSOR);

//     int target = 0;

//     PID armPID(0.5, 0.0, 0.0, 0, 100, 0);

//     pros::Task *moveTask = nullptr;


//     void init()
//     {
//         armMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//         armSensor.set_position(0);
//         pros::delay(200);
//         std::cout << "Init pos: " << armSensor.get_position() << std::endl;
//         pros::Task liftControlTask([]{
//             while (true) {
//                 liftControl();
//                 pros::delay(5);
//             }
//         });
        
//     }

//     void liftControl()
//     {
//         double error = target - armSensor.get_position();
//         double dt = 0.005; // 5ms
//         double output = armPID.update(error, dt);
//         armMotor.move(output);
//     }

//     void setTarget(int target)
//     {
//         target = target;
//     }

// }