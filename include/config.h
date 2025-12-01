#ifndef CONFIG_H
#define CONFIG_H

#include "main.h"

#define FORWARD_CONTROL pros::E_CONTROLLER_ANALOG_LEFT_Y // for arcade drive
#define YAW_CONTROL pros::E_CONTROLLER_ANALOG_RIGHT_X     // for arcade drive
#define RIGHT_CONTROL pros::E_CONTROLLER_ANALOG_RIGHT_Y  // for tank drive
#define LEFT_CONTROL pros::E_CONTROLLER_ANALOG_LEFT_Y    // for tank drive

#define LEFT_BOT_MOTOR 1
#define LEFT_MID_MOTOR 3
#define LEFT_TOP_MOTOR 2
#define LEFT_MOTOR 2

#define RIGHT_BOT_MOTOR 12
#define RIGHT_MID_MOTOR 9
#define RIGHT_TOP_MOTOR 15
#define RIGHT_MOTOR 1


#define IMU_PORT 18
#define FIRST_STAGE 20
#define SECOND_STAGE 4

#define ARM_MOTOR 4
#define ARM_SENSOR 3


#define SAMPLE_PISTON 'H'

#define TRACKING_PORT 19

#endif