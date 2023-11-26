#include <stdio.h>
#include "robot-config.h"

using namespace vex;

robotChasis::robotChasis( float wD){
  wheelDiameter = wD;
  leftTracker.resetPosition();
  rightTracker.resetPosition();
}

void robotChasis::set_drive_break_type(brakeType B){
  LeftRear.setBrake(B);
  LeftMid.setBrake(B);
  LeftFront.setBrake(B);
  RightRear.setBrake(B);
  RightMid.setBrake(B);
  RightFront.setBrake(B);
}

void robotChasis::stopMotors(){
  LeftRear.stop();
  LeftMid.stop();
  LeftFront.stop();
  RightFront.stop();
  RightMid.stop();
  RightRear.stop();
}

double robotChasis::getPI() { return PI; }
float robotChasis::getsL() { return sL; }
float robotChasis::getsR() { return sR; }
float robotChasis::getsS() { return sS; }
double robotChasis::getWheelCir(){ return PI * wheelDiameter; }

void vexcodeInit(robotChasis *robot1) {
  robot1->leftTracker.resetPosition();
  robot1->rightTracker.resetPosition();
  wait(10, msec);
  
   //Gyro Callibrates
  robot1->gyroM.calibrate();
  while(robot1->gyroM.isCalibrating()){
    wait(25, msec);
  }

  // Prints the values of the tracking wheels in degrees.
  //printf("%.0lf, %.0lf, %.0lf \n", robot1->leftTracker.position(deg), robot1->rightTracker.position(deg), robot1->backTracker.position(deg));
}