#include <stdio.h>
#include "robot-config.h"

robotChasis::robotChasis( float wD, float tcL, float tcR, float tcB){
  wheelDiameter = wD;
  sL = tcL;
  sR = tcR;
  sS = tcB;

  fly1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  fly2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
  rightTracker.reset_position();
  leftTracker.reset_position();
  backTracker.reset_position();

  
}

void robotChasis::set_drive_break_type(pros::motor_brake_mode_e_t B){
  leftFront.set_brake_mode(B);
  leftMid.set_brake_mode(B);
  leftRear.set_brake_mode(B);
  rightFront.set_brake_mode(B);
  rightMid.set_brake_mode(B);
  rightRear.set_brake_mode(B);
}

void robotChasis::stopMotors(){
  leftFront.move(0);
  leftMid.move(0);
  leftRear.move(0);
  rightFront.move(0);
  rightMid.move(0);
  rightRear.move(0);
}

double robotChasis::getPI() { return PI; }
float robotChasis::get_flbr() { return flbrWheels; }
float robotChasis::get_frbl(){ return frblWheels; }
float robotChasis::getsL() { return sL; }
float robotChasis::getsR() { return sR; }
float robotChasis::getsS() { return sS; }
double robotChasis::getWheelCir(){ return PI * wheelDiameter; }

void vexcodeInit(robotChasis *robot1) {
  robot1->leftTracker.reset_position();
  robot1->rightTracker.reset_position();
  robot1->backTracker.reset_position();

  pros::delay(500);
  // Gyro Callibrates
  /*robot1->gyroM.calibrate();
  while(robot1->gyroM.isCalibrating()){
    wait(50, msec);
  }*/

  // Prints the values of the tracking wheels in degrees.
  printf("%.0lf, %.0lf, %.0lf \n", robot1->leftTracker.get_position()*100, robot1->rightTracker.get_position()*100, robot1->backTracker.get_position()*100);
}