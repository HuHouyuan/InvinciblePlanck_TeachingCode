/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/


#include "autonomous.h"
#include "task.h"
#include "autopath.h"
#include<iostream>
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>

using namespace vex;
using namespace Eigen;  
// A global instance of competition
competition Competition;

// Vector2f getLocalSpeed(){
//   xpos.resetRotation();
//   ypos.resetRotation();
  
  
//   Vector2f initPos;
//   Vector2f finalPos;
//   initPos<< 0, 0;
//   finalPos<< 0, 0;
//   Vector2f result;

//   while(1){
//       finalPos<<-xpos.rotation(deg) , -ypos.rotation(deg);
//       result = finalPos - initPos;
//       Brain.Screen.printAt(10,40,"LocalSpeed:%.2f %.2f",result[0],result[1]);
//       initPos = finalPos;
//       wait(10, msec);   
//   }
// }

// Vector2f getOmniPos(){
//   xpos.resetRotation();
//   ypos.resetRotation();
//   mygyro.setRotation(90,deg);
//   float d;

//   Vector2f initPos;
//   Vector2f finalPos;
//   initPos<< 0, 0;
//   finalPos<< 0, 0;
//   Vector2f result;
//   float rx = 0;
//   float ry = 0;
//   while(1){
//     d=90-mygyro.rotation();
//       finalPos<<-xpos.rotation(deg), -ypos.rotation(deg);
      
//       result = SpinMatrix(d)*(finalPos - initPos);
//       rx += result[0];
//       ry += result[1];
//       Brain.Screen.printAt(10,40,"OmniPos:%.2f %.2f",rx,ry);
//       initPos = finalPos;
//       wait(10, msec);   
//   }
// }

void runAuto(){
  int a = Brain.timer(msec);
  super_intake.set(0);
  puncher_air.set(0);
  inAuto = 1;
  special = 0;
  inManualControl = 0;
  switch (autoRoutine)
  {
  case 0:
    break;
  case 1 :
    if(!afraid)
      one();
    else
      one_afraid();
    break;
  case 2:
    two();
    break;
  case 3:
    three();
    break;
  case 4:
    four();
  default:
    break;
  } 
  cout << "time used: "<<Brain.timer(msec)-a <<endl;
  inAuto = 0;
}

void autonomous(void) {
  runAuto();
  
}

void usercontrol(void) {
  // User control code here, inside the loop
  fifteen = 0;
  super_intake.set(0);
  puncher_air.set(0);
  expansion.set(0);
  inAuto = 0;
  //Vector2f testv = Vector2f(60, 60);

  while (1) {
    //if(BX) test1();
    ch_move();
    intakeAndRoll();
    if (!lRight && RIGHT) autoRoutine = (autoRoutine == 5)? 1:autoRoutine+1;
    

    if(R1&&!lR1) readyToPunch = 1;
    if(R2){
      PIDGyroTurn(mygyro.rotation()+LookAt(0, shootPoint));
    }
    // if(BY&&!lBY){ 
    //   expansion.set(1);
    // }



    if(UP) driveForward(45);
    else if (DOWN) driveForward(-25);
    else if (LEFT) driveRotate(-25);
    else if (RIGHT) driveRotate(45);
    else driveForward(0);

    if(BA&&!lBA){
      //manualPuncherSet = (manualPuncherSet == 2)? 0:manualPuncherSet+1;
      GPS_move(-130, 50, globalRot);
    }
    if(BB&&!lBB){
      GPS_move(-80, 150, globalRot);
    }
    if(BY&&!lBY){
      GPS_move(-30, 220, globalRot);
    }
    // if(LEFT&&lLEFT){
    //   inAuto = 1;
    //   driveForward(100);
    // }
    
    lR1 = R1;
    lBY =BY;
    lUP = UP;
    lRight = RIGHT;
    lDown = DOWN;
    lLEFT = LEFT;
    lBA = BA;
    lBB = BB;
    delay(20); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//='//
// Main will set up the competition functions and callbacks.
//
int main() {
  delay(200);
  Controller1.Screen.clearScreen();
  //Controller1.Screen.clearScreen();
  // Set up callbacks for autonomous and driver control periods.
  super_intake.set(0);
  puncher_air.set(0);
  expansion.set(0);
  //task GETGYROREADING(getGyroReading);
  
  task UPDATECOR(updateCOR);
  task VAC(vac);
  task PRINTSTUFF(printStuff);
  task UPDATEGLOBALROT(updateGlobalRot);
  task SHOOTJUDGER(ShootJudger);
  task PUNCHERENCCONTROL(PuncherEncControl);
  task PUNCHERJUDGER(PuncherJudger);
  task GIVESPEEDINAIM(giveSpeedInAim);
  delay(200);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (true) {
    delay(100);
  }
}
