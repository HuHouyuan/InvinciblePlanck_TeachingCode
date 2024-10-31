#ifndef _CONFIG_
#define _CONFIG_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

using namespace vex;
using namespace std;
brain Brain;
#undef __ARM_NEON__
#undef __ARM_NEON

/* this line of code defines the controller used for controlling the robot */
controller Controller1 = controller(primary);

/* this is how you define motors. There are three parameters for the constructor: port number, gear setting, and whether
reverse the default positive direction of the motor */
/* For the port number, enter the port number of the brain where you connect the motor to*/
/* For the gear setting, enter ratio6_1, ratio18_1, or ratio36_1 according to the motor used. Enter ratio6_1 if the red motor is used，
ratio18_1 if the green motor is used, and ratio6_1 if the blue motor is used. If you are not sure which motor is used, ask the builder*/
/* Motors have a default positive direction, which you can obtain my looking at the motor box. If you want the default direction
to stay the same with the direction indicated on the box, enter false. Otherwise if you want the default direction to be the opposite one,
enter true */
motor LF = motor(PORT1, ratio6_1, 0); // motor for the left front wheel
motor LB = motor(PORT2, ratio6_1, 0); // motor for the left back wheel
motor RF = motor(PORT3, ratio6_1, 0); // motor for the right front wheel
motor RB = motor(PORT4, ratio6_1, 0); // motor for the right back wheel

/* this is how you define the gyroscrope. The gyro is used for obtaining the orientation of the robot car */
inertial Gyro = inertial(PORT5);

/* After finish reading this file, you may move on to the main function in main.cpp*/

void vexcodeInit(void);

#endif
