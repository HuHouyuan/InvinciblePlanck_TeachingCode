#ifndef _AUTO_
#define _AUTO_
#include "declaration.h"
using namespace vex;

// this function takes the input value and set the velocity for the robot to move forward/backward
void driveForward(float v)
{
  v *= 120;
  LB.spin(fwd, -v, vex::voltageUnits::mV);
  LF.spin(fwd, -v, vex::voltageUnits::mV);
  RB.spin(fwd, v, vex::voltageUnits::mV);
  RF.spin(fwd, v, vex::voltageUnits::mV);
}

// this function takes the input value and set the velocity for the robot to rotate right/left
void driveRotate(float v)
{
  v = v * 120;
  LB.spin(fwd, -v, vex::voltageUnits::mV);
  LF.spin(fwd, -v, vex::voltageUnits::mV);
  RB.spin(fwd, -v, vex::voltageUnits::mV);
  RF.spin(fwd, -v, vex::voltageUnits::mV);
}

/* this function takes the input value and set the velocity for the robot to move forward/backward for a set of time
   timerForward is often used to make sure the robot moves to a point, especially reaching some static objects like a wall. */
/* the timer() returns the time recorded on the brain*/
void timerForward(float v, float t)
{
  float initT = Brain.timer(msec);
  while (Brain.timer(msec) - initT < t)
  {
    driveForward(v);
  }
  driveForward(0);
}

void timerRotate(float v, float t)
{
  float initT = Brain.timer(msec);
  while (Brain.timer(msec) - initT < t)
  {
    driveRotate(v);
  }
  driveRotate(0);
}

/* Encoder is a thing that tracks the revolution of the motor */
/* this function takes the input value and set the velocity for the robot to move forward/backward
   until the revolution reaches the input value */
/* encoderForward has a higher accuracy than timerForward, but not as accurate as PIDForward, which will be discussed very soon */
void encoderForward(float v, float target)
{
  float enc = (LF.position(deg) + LB.position(deg)) / 2;
  float iniT = Brain.timer(msec);
  while (fabs((LF.position(deg) + LB.position(deg)) / 2 - enc) < fabs(target))
  {
    driveForward(v);
    if (Brain.timer(msec) - iniT > 3000)
      break;
  }
  driveForward(0);
}

/* The PID algorithm enables incrediblely accurate movement/rotation. You will have a more comprehensive
   class about this algorithm during your trainings. The following is just for you to first briefly understand
   waht PID does.

   Let's first discuss why encoderForward is not accurate enough. Let's say you want the car
   to move 1000 encoder units forward with a speed of 100%. Before the car reaches the target 1000,
   it would always travel with 100% speed. When it reaches the target 1000, the car sets the speed to 0.
   However, due to inertia, it will keep move forward for a while. So the car ends up moving more than 1000
   encoder units. In order to solve this problem, a PID algorithm is needed. Note that PID algorithm can be used
   in all kinds of functions that require the robot to accurately perform a certain task, and it's not limited
   to locomotion only.

   There are three steps needed to make the car move accurately.
   The first step is P, which stands for proportion. When the car moves more close to the target,
   we make the velocity lower. The slope for this change is defined using the constant Kp. Larger
   the Kp, slower the velocity is reduced as it reaches the target.
   In the ideal situation, when the car reaches the target, we wish the velocity is just
   reduced to zero.However, in reality, it's not possible. Although we can set Kp very small, so that
   the car moves in a very slow velocity and eventually will reach the target accurately, it's too slow.
   We want the car to move fast and accurate at the same time. So, in order to make the car move fast,
   the Kp can not be set to a very small value. But when we do this, inertia would still affect the locomotion
   process, resulting in the car exceeding the target, though the exceeded value would be smaller than the exceede
   value using encoderForward. Moreover, due to Kp, the car would know to move backward when it exceeds the target.
   So, when there's only Kp, we would see the car oscillating back and forth from the target point,
   eventually reaching the target point with precision.

   In order to reduce the times of oscillation, the second step, D, which stands for derivative, comes into play.
   The idea is that, we want to further reduce the velocity of the car if the car's velocity is too large when it's
   near to the target. The larger the velocity, the more we want the velocity to reduce, preventing exceeding the target.
   As Kp increases, the velocity decreases more intensely. By adding Kd, the time of oscillation will greatly reduce.

   However, sometimes you may find the car "stop" infront of the target location. This is due to the forward force given
   by the motor is canceled out by the friction force, making the car unable to keep moving forward. How the I term exactly works
   is complicated, and you may find that you do not need to use the I term in certain cases. Simply put, it makes the car gain an
   instant forward acceleration when the car is infront of the target location. How the I term exactly works will be discussed
   during trainings.

   Finally, we have to make the car stop when it accurately reaches the desired location. There are two conditions need to be met before
   the PID stops. First, the error between the car's current displacement and the target displacement has to be smaller than a specific value,
   which is called tolerance. Second, the velocity of the car also have to be small enough, so it will not be affected by inertia significantly
   even after PID stops. When these two conditions are met, we can thinkg that the car has accurately reached the target location.

   Note that in the function given, we do not assign velocity when using the PID algorithm. This is because the PID algorithm is supposed
   to calculate the velocity by itself. In more advanced PID algorithms, you can add a parameter called v_limit which limits the maximum
   velocity for the car if you want the car to move in a slow speed. We will go over the more advanced model of PID during trainings and
   learn more about other improvements to the PID algorithm, such as detecting the current time used to see if the robot is stuck at a place
   or using slow incremement during the beginning of the movement to restrain the car's starting velocity.
   */

void PIDForward(float target, float tolerance = 30)
{
  // setting value for Kp, Ki, and Kd. Note that different cars have (very) different sets of Kp, Ki, and Kd.
  float kp = 3;
  float ki = 1;
  float kd = 30;
  /*how far infront of the target displacement do you want Ki to participate. Note that the I term gets
  larger and larger after every loop, and you do not want the I term to participate into the controlling process too soon */
  float istart = 100;
  // tolerance for the velocity
  float dtol = 1.8;
  float errortolerance = tolerance;
  // obtaining the current displacement
  float enc = (RF.position(deg) + RB.position(deg)) / 2;
  float error = 0;
  float lasterror = error;
  float v = 0;
  float i = 0;
  bool arrived = false;
  float pow = 0;
  while (!arrived)
  {
    // updating the current displacement
    error = target - ((RF.position(deg) + RB.position(deg)) / 2 - enc);
    // obtaining the velocity of the current robot car
    v = LF.velocity(dps) / 100;
    // checking if the two conditions for accurate arrival are met
    if ((fabs(error) < errortolerance && fabs(v) <= dtol)

    )
    {
      arrived = true;
    }

    // When the car reaches istart, we let the I term to participate
    if (fabs(error) < istart)
      i += error;
    else
      i = 0;
    if (error * lasterror <= 0)
    {
      i = 0;
    }

    // combining the P, I, and D term to get the final ouput velocity for the car
    pow = kp * error + kd * v + ki * i;
    driveForward(pow);
    lasterror = error;
    wait(10, msec);
  }
  driveForward(0);
}
/* When do we use encoderForward and when do we use PIDForward?
   Although PIDForward is more accurate, the time needed for it to reach the target is longer.
   So, use encoderForward during short movements and the error will not greatly affect the performance
   Use PIDForward when you want accurate locomotion or during long distance movements.
   However, despite the advantages regarding time usage of encoderForward, PIDForward is often used in most
   cases, since we do not want errors to be too large in most cases. */

/* this is the PID algorithm for rotation after finishing this part, you may move on to the autopath.h file to see
how we actually code an autonomous path for the robot */
/* Note that right is the positive direction of rotation for the gyro. And, when using this PIDGyroTurn,
instead of making the car to rotate a specific angle, we let the car to turn to a specific angle. This approach
is better because it reduces error accumulatio (explanation: although PID is already very accurate, it still
makes insignificant errors after each action. So, for example, after the robot rotates many times, the small errors generated from
each PIDGyroTurn will accumulate, making the accumulated error significant enough to affect the automation. However, by making the robot
turn to a specific direction, error accumulation can be greatly reduced). Another brilliant way to reduce error accumulation
during locomotion is using a complex algorithm called odometry. You will briefly learn this algorithm during the first few trainings,
but mastering it and coding your own odometry algorithm may take lots of months and even years. */
void PIDGyroTurn(float target, float tolerance = 1)
{
  float kp = 3;
  float ki = 1;
  float kd = 30;
  float istart = 8;
  float dtol = 0.5;
  float errortolerance = tolerance;
  float error = target - Gyro.rotation(deg);
  float lasterror = error;
  float v = 0;
  float i = 0;
  bool arrived = false;
  float pow = 0;
  while (!arrived)
  {
    error = target - Gyro.rotation();
    v = Gyro.gyroRate(zaxis, dps) / 100;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol)

    )
    {
      arrived = true;
    }
    if (fabs(error) < istart)
      i += error;
    else
      i = 0;
    if (error * lasterror <= 0)
    {
      i = 0;
    }
    pow = kp * error + kd * v + ki * i;
    driveRotate(pow);
    // cout << pow << "   " << error << "    " << v << "   " << i<< endl;
    //  printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    wait(10, msec);
  }
  // cout << timeused << endl;
  driveRotate(0);
}

#endif