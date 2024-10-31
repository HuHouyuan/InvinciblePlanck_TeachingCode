/*
Author: Morgan Hu

Welcome to Invincible Planck! I'm pretty sure you will have a great time figuring out various algorithms and coorperating with your friends!
When you are reading this document for the first time and wish to learn the basic functions & algorithms used in VEX Robotics,
first go to the robot-config.h file :) Note that the wait function literally makes the code stop for the assigned time.
*/
#include "robot-config.h"
#include "declaration.h"
#include "autopath.h"
using namespace vex;

competition Competition;

int autoRoute = 1;
/* Before every match, we want to choose the desired autonomous route, and each route corresponds
   to a seperate function in the autopath.h file. Before going to the autopath.h file, you may first
   move on to the autonomous function and check out the common functions & algorithms used for the autonomous period */
/* Note that code for switching autoRoute is in usercontrol function*/
void autonomous(void)
{
  int a = Brain.timer(msec);
  switch (autoRoute)
  {
  case 1:
    one();
    break;
  case 2:
    two();
    break;
  default:
    break;
  }

  // every autonomous period is 15 seconds, remember do not exceed this time limit.
  cout << "time used: " << Brain.timer(msec) - a << endl;
}

void usercontrol(void)
{
  bool lastBA = 0;
  while (1)
  {
    /* the following code enables users to control the movement of the car through using the left stick */
    /* the spin function is the function that allows the motor to spin with the input direction and velocity.
       the maximum velocity is 120 * 100, and inputing a negative number for the velocity will make the motor
       spin in the opposite direction */
    /* Note that we often use Macro for convenience during coding. You may check the Macro in the declaration.h file */
    /* Ch3, located on the left stick, is often used by us for moving forward or backward. Ch1, located on the right stick,
       is often used by us for rotating */
    LB.spin(fwd, (-Ch3 + Ch1) * 120, vex::voltageUnits::mV);
    LF.spin(fwd, (-Ch3 + Ch1) * 120, vex::voltageUnits::mV);
    RB.spin(fwd, (Ch3 + Ch1) * 120, vex::voltageUnits::mV);
    RF.spin(fwd, (Ch3 + Ch1) * 120, vex::voltageUnits::mV);

    /* Here are two examples of how to correctly use buttons
    1st case: call a function if a button is constantly pressed
    for example, checking if button A is constantly pressed, if yes, a certain function may be called. BA is also a Macro, check other
    Macro for other buttons in the declaration.h file. lastBA checks if button A is pressed in the last while loop. remember always to update
    the status of lastBA at the end of every while loop
    if(BA && lastBA) {
      function();
    }

    2st case: call a function if a button is released after pressing
    You may wonder why we need to call a function after a button is released after pressing. Think about the following scenario:
    let's say the task of the year is to shoot balls into a basket, and you want the user to be capable of manually changing the
    robot's shooting range. Let's say there are three shooting range available, close, middle, and far. After pressing the button A,
    the shooting range changes to the next level, such as changing from close to middle, or from far to close. If you just write the
    following code:

    if(BA) {
      function();
    }

    you will find that the shooting range has changed more than once after the button is pressed. This is because when humans press buttons, we inevitably press
    it for a longer time then the time of each while loop (20msec). So, even if you think you just pressed the button once, the code
    thinks that you have pressed the button mulitple times. Thus, in order to solve this problem, the best approach is to call the function
    if a button is released after pressing
    if(BA && !lastBA) {
      function();
    }
    */

    /* Here is a demonstration of using buttons. You do not need to know what autoRoute stands for currently, as this will be covered
    as you finish reading other parts of this file */
    if (BA && !lastBA)
    {
      autoRoute = autoRoute + 1 > 2 ? 0 : autoRoute + 1;
    }

    /* After finishing this part, you may move on to the autonomous function to see how we control the bot to act automatically */
    lastBA = BA;
    wait(20, msec);
  }
}

int main()
{
  // Set up callbacks for autonomous and driver control periods.
  /* The drivecontrol function is responsible for the user control period,
  while the autonomous function is responsible for the autonomous period */
  /* You may first move on to the usercontrol control to check out the code for moving, rotating, and button pressing detection */
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(autonomous);

  while (true)
  {
    wait(100, msec);
  }
}
