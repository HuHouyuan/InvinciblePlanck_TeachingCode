#ifndef AUTOPATH
#define AUTOPATH

#include "robot-config.h"
#include "declaration.h"
#include "autonomous.h"

void one()
{
    /* let's assume route 1 does the following actions. Here are some simple tests for you :)
    1. Test yourself and see if you can understand every step of route 1
    2. Write your own route 2 according to the prompt in the two function*/

    // assume when the robot starts this route, it is in a plain field with no obstacles and the gyro is 0 degrees.
    /* note that after using timerForward and encoderForward, we want the car to wait a few msec so that the velocity is reduced to 0
    and will not affect the next action */
    PIDForward(500);
    PIDGyroTurn(90);
    timerForward(40, 1000);
    wait(300, msec);
    PIDGyroTurn(0);
    encoderForward(-30, 200);
    wait(300, msec);
    PIDGyroTurn(-90);
    PIDForward(-2000);
    timerForward(-10, 1000);
}

void two()
{
    /* Now try and write your own route!
       assume when the robot starts this route, it is in a plain field with no obstacles and the gyro is 0 degrees.

       1. the car turns to the 45 degree direction
       2. the car moves 1500 encoder units forward
       3. the car turns 90 degrees to the left.
       4. the car moves 200 encoder units backward in any given speed. Having error at this step will significantly affect the performance
       5. the car further moves back in a velocity of 30% for 1 second
       6. the car turns to the 0 degree direction
       7. the car moves 300 encoder units forward in maximum speed possible. Having an error at this step will not
       significantly affect the performance and we want this step to be as fast as possible
       8. the car turns 180 degrees to the right */
}
#endif