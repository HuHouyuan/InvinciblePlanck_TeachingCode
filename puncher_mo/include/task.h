#ifndef _TASK_
#define _TASK_
#include "declaration.h"
#include "robot-config.h"
#include <iostream>
#undef __ARM_NEON__
#undef __ARM_NEON
#include <eigen-3.4.0/Eigen/Dense>
using namespace Eigen; 

float vch3 = 0;
float vch4 = 0;
float vch1 = 0;

bool lL1 = 0;
bool lR1 = 0;
bool lR2 = 0;
bool lL2 = 0;
bool lBX = 0;
bool lBY = 0;
bool lBB = 0;
bool lUP = 0;
bool lBA = 0;
bool lRight = 0;
bool lDown = 0;
bool lLEFT = 0;
bool isGod = 0;
bool isAim = 0;
bool isOpen = 0;
bool currentBool = 0;
bool special = 0;
bool criticalAdd = 0;
bool ManualShoot = 0;
bool chooseRed = 0;
bool chooseBlue = 0;
bool fifteen = 0;
bool notBad = 0;
bool afraid = 0;
int manualPuncherSet = 0;


void punch(){
  currentBool = inAuto;
  //打开盖子
  inAuto = 1;
  intake_setspeed(-100);
  delay(300);
  //打开气动
  puncher_air.set(1);
  puncher_setspeed(-5);
  cout<<puncherTarget<<endl;
  delay(200);
  readyToPunch = 0;
  puncher_air.set(0);
  delay(400);
  inAuto = currentBool;
  //拉回
  puncher_setspeed(5);
  delay(550);
  leftPuncher.resetPosition();
  if(inAuto){
    intake_setspeed(100);
  }
  else{
    intake_setspeed(100);
    delay(100);
    intake_setspeed(30);
    delay(200);
    intake_setspeed(0);
  }
}


int PuncherJudger(){
  delay(100);
  //
  leftPuncher.resetPosition();
  readyToPunch = 0;
  float myTime = Brain.timer(msec);
  /*
  while(1){
    //check if the puncher can punch and the controller pressed the buttom of punching
    if(readyToPunch) punch();
    //if the controller isn't going to punch, adjust the position of the puncher according to the 
    //current position
    else{
      puncher_setspeed(3*(puncherTarget - leftPuncher.position(deg))+10);
      //cout<< "setspeed " <<3*(puncherTarget - leftPuncher.position(deg))+5<<endl;
    }

    // if(myTime > 56*1000 && !notBad){
    //   GPS_move(120, 120, 225+360);
    //   expansion.set(1);
    // }
    delay(10);
  }
  */
  return 0;
}

double distance2encoder(double x)
{
  double y =   -0.0000186435 * pow(x,3) + 0.0129697 * pow(x,2) - 0.669693 * x + 630.031; //578

  
  //420.388 + 2.9685*x - 0.00747167 * pow(x,2) + 0.000015325 * pow(x,3); //370.388 // 400
  if (y > 1050){
    return 1050;
  } 
  else{
    return y;

  }
}

int ShootJudger(){
  delay(100);
  float toRed = 0;
  float toBlue = 0;
  while(1){
    toRed = Vector2f( redShoot[0] - corX, redShoot[1] - corY).norm();
    toBlue = Vector2f( blueShoot[0] - corX, blueShoot[1] - corY).norm();
    if(fifteen){
      shootPoint = Vector2f(-132, 132);
    }
    else if(ManualShoot){
      if(chooseBlue) shootPoint = blueShoot;
      else shootPoint = redShoot;
    }
    else{
      if(toRed<toBlue) shootPoint = redShoot;
      else shootPoint = blueShoot;
    }
    
    delay(10);
  }
  return 0;
}

int PuncherEncControl()
{
  delay(100);
  dis = 0;
  puncherTarget = 600;
  while (1) 
  {
    dis = Vector2f( shootPoint[0] - corX, shootPoint[1] - corY).norm();
    if(manualPuncherSet == 0)
      puncherTarget = distance2encoder(dis);
    else if(manualPuncherSet == 1)
      puncherTarget = 660;
    else 
      puncherTarget = 830;
    // if(BA&&!lBA) puncherTarget+=10;
    // if(BY&&!lBY) puncherTarget-=10;
    // if(DOWN&&!lDown) puncherTarget+=5;

    // cout<<"puncherTarget: "<<puncherTarget<<endl;
    // cout<<"nowpuncher: "<<leftPuncher.position(deg)<<endl;
    // lBA = BA;
    // lBY = BY;
    // lDown  = DOWN;
    delay(10);
  }
  return 0;
}

int getGyroReading(){
  while(1){
    Controller1.Screen.setCursor(2, 1);
    //Controller1.Screen.print(Gyro.rotation());
  }
  return 0;
}

int updateGlobalRot(){
  delay(100);
  globalRot = 180 - IGR;
  float initGyro = mygyro.rotation();
  float finalGyro = initGyro;
  float dG = 0;
  while(1){
    //cout<<"deltaGyro: "<<dG<<" ||globalRot: "<<globalRot<<endl;
    finalGyro = mygyro.rotation();
    dG = finalGyro-initGyro;
    globalRot -= dG;
    initGyro = finalGyro; 
    delay(10);
  }
  return 0;
}

Vector2f result;
int updateCOR() {
  delay(100);
  xpos.resetRotation();
  ypos.resetRotation();
  mygyro.setRotation(IGR, deg);
  float cor;
  
  Vector2f dis;
  dis << 0, 16;
  float d;
  Vector2f initPos;
  Vector2f finalPos;
  initPos << 0, 0;
  finalPos << 0, 0;
  float initGyro = 90;
  float finalGyro;
  corX = 0;
  corY = 0;
  // rx = 135.25;
  // ry = 34.3;
  rx = 0;//-75.25;
  ry = 0;//-145.7;
  while (1) {
    finalGyro = IGR - mygyro.rotation();
    d = (initGyro + finalGyro) / 2;
    cor = IGR - mygyro.rotation();
    finalPos << -xpos.rotation(deg), -ypos.rotation(deg);
    result = SpinMatrix(d) * (finalPos - initPos);
    result = SpinMatrix(45) * result;
    rx += result[0] * xratio;
    ry += result[1] * yratio;
    corX = rx + (SpinMatrix(cor) * dis)[0];
    corY = ry + (SpinMatrix(cor) * dis)[1];
    globalspeed = Vector2f(result[0]*xratio/100, result[1]*yratio/100);
    initPos = finalPos;
    initGyro = finalGyro;
    delay(10);
  }
  return 0;
}

float LookAt(float direc, Vector2f target){
  Vector2f toTarget;
  //get the target direction which the robot needs to rotate to
  toTarget << target[0]-corX, target[1]-corY;
  Vector2f StandardC;
  Vector2f now;
  float dc;
  float Deg;
  float isDeg;
  StandardC<< 0, 1;
  dc = IGR - mygyro.rotation();
  now = SpinMatrix(dc) * StandardC;
  //calculate the angle between the target facing vector and current facing vactor
  Deg = acos(toTarget.dot(now)/(now.norm()*toTarget.norm()));
  Deg = Deg/M_PI * 180;
  Vector2f isResult = SpinMatrix(-Deg) * now;
  //check if the direction of the turn is correct or not, if not, set it to the opposite direction
  isDeg = acos(isResult.dot(toTarget)/(isResult.norm()*toTarget.norm()));
  isDeg = isDeg/M_PI*180;
  if (isDeg>3) Deg = -Deg;
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1, 1);
  // Controller1.Screen.print(Deg);
  return Deg;
}

float getSpeed(){
  return sqrt(result[0]*result[0] + result[1]*result[1]);
}


int giveSpeedInAim(){
  delay(100);
  float target = mygyro.rotation() + LookAt(0, shootPoint);
  Vector2f fakeShootPoint = shootPoint;
  float tolerance = 0.5;
  float timeroffset = TIMER;
  float timeused = 0;
  float kp;
  float ki;
  float kd;
  float imin = 10; // ji fen fan wei
  float istart = 60;     // start to integral
  float dtol = 0.2;
  float errortolerance = tolerance; // 2.5zd
  float lim = 100;
  float error = target - Gyro.rotation(deg);
  float lasterror;
  float v = 0;
  float i = 0;
  bool arrived, firstOver = true;
  float timetol = fabs(error) < 20 ? 700 : fabs(error) * 24;
  lasterror = error;
  arrived = error == 0;
  float Kdistance = 3;
  float predis = sqrt((shootPoint[0]-corX)*(shootPoint[0]-corX)
                     +(shootPoint[1]-corY)*(shootPoint[1]-corY));
  fakeShootPoint = shootPoint - globalspeed*(16.3+predis*0.1);
  while (1) {
    
    predis = sqrt((shootPoint[0]-corX)*(shootPoint[0]-corX)
                     +(shootPoint[1]-corY)*(shootPoint[1]-corY));

    target = mygyro.rotation() + LookAt(0, fakeShootPoint);
    if(arrived) aim_ch1 = 0;
    if(getSpeed()<1.2&&fabs(error)<0.5){
      fakeShootPoint = shootPoint;
      kp = 5.5;
      ki = 0.2;
      kd = 55;
    }
    else{
      fakeShootPoint = shootPoint - globalspeed*(16.3+predis*0.2);
      kp = 5.5;
      ki = 0;
      kd = 47;
    }
    timeused = TIMER - timeroffset;
    error = target - Gyro.rotation();
    // v = (error - lasterror) / dt;
    v = -Gyro.gyroRate(zaxis, dps) / 100;
    if ((fabs(error) < errortolerance && fabs(v) <= dtol)
        //||timeused > timetol
    ) {
      arrived = true;
    }
    if (fabs(error) < istart)
      i += sgn(error);
    if (error * lasterror <= 0) {
      if (firstOver) {
        i = sgn(error) * imin;
        firstOver = false;
      } else {
        i = 0;
      }
    }

    aim_ch1 = kp * error + kd * v + ki * i;
    aim_ch1 = fabs(aim_ch1) > lim ? sgn(aim_ch1) * lim : aim_ch1;
    //pow = fabs(pow) > slow ? sgn(pow) * slow : pow;
    // printScreen(10,100,"Iner %f",Iner.rotation());
    lasterror = error;
    delay(10);
  } 
  return 0;
}

int vac(){
  delay(100);
  while(1){
    if(BB&&!lBB){
      isGod = !isGod;
    }
    if(BX&&lBX){
      isAim = 1;
    }
    else{
      isAim = 0;
    }
    lR2 = R2;
    lBB = BB;
    delay(10);
    
  }
}

void normalMove() {
  vch3 = Ch3;
  vch4 = Ch4;
}

void godPerspective(){
  Vector2f GodV;
  //get the input from the controller
  GodV << Ch4, Ch3;
  //spin the vector to the third person persepctive
  GodV = SpinMatrix(mygyro.rotation()-90) * GodV;
  //set the speed
  vch4 = GodV[0];
  vch3 = GodV[1];
}

Vector2f preaim;
void Aim(){
  vch1 = aim_ch1;
}
void notAim(){
  vch1 = Ch1;
}


void ch_move(){
    
      if(!isGod) normalMove();
      else godPerspective();
      if(!isAim) notAim();
      else Aim();
    //if(!inAuto){
      LF.spin(fwd, (vch3 + vch4 + vch1) * 120, vex::voltageUnits::mV);
      LB.spin(fwd, (vch3 - vch4 + vch1) * 120, vex::voltageUnits::mV);
      RF.spin(fwd, (-vch3 + vch4 + vch1) * 120, vex::voltageUnits::mV);
      RB.spin(fwd, (-vch3 - vch4 + vch1) * 120, vex::voltageUnits::mV);
    //}
    delay(10);
}

void intakeAndRoll(){
  if((L1&&lL1)||(L2&&lL2)){
    if(L1&&lL1){
      //cout<<"inL1"<<endl;
     intk.spin(fwd, 120*100, vex::voltageUnits::mV);
     roller.spin(fwd, 100 * 120, vex::voltageUnits::mV);
    }
    else{
    //  cout<<"inL2"<<endl;
      roller.spin(fwd, -100 * 120, vex::voltageUnits::mV);
      intk.spin(fwd, -100 * 120, vex::voltageUnits::mV);  
    }
}
else if (!inAuto){
  //cout<<"bad:("<<endl;
    roller.spin(fwd, 0, vex::voltageUnits::mV);
    intk.spin(fwd, 0, vex::voltageUnits::mV);
}
  lL1 = L1;
  lL2 = L2;
}

int printStuff(){
  delay(100);
  int a;
  int b;
  while(1){
    a = isGod? 1 : 0;
    b = isAim? 1 : 0;
    Brain.Screen.printAt(10, 40, "COR:%.2f %.2f",
                         corX ,
                         corY );
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("P: %d||A: %d||G: %.2f", a, b, mygyro.rotation());
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("X: %.2f||Y: %.2f", corX, corY);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("Routine: %d||Afraid: %d", autoRoutine, afraid);

    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("LF: %.2f||LB: %.2f", LF.position(deg), LB.position(deg));
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("RF: %.2f||RB: %.2f", RF.position(deg), RB.position(deg));

    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("Gyro: %.2f",mygyro.rotation());
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print(autoRoutine); 

    // Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("leftEncoder: %.2f", leftPuncher.position(deg));
    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("dis: %.2f", Vector2f(shootPoint[0]-corX,shootPoint[1]-corY).norm());
    // Controller1.Screen.setCursor(3,1);
    // Controller1.Screen.print("puncherTarget: %.2f", puncherTarget); 
    delay(10);
  }
}
#endif