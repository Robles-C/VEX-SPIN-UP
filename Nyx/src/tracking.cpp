#include "tracking.h"


odometry::odometry(robotChasis *robot, double x, double y, double deg){
  robot1 = robot;
  xPos = x;
  yPos = y;
  angleD = deg;
  angleR = (deg*robot->getPI())/180;
}

double odometry::getangleD(){ return angleD; }
double odometry::getangleR(){ return angleR; }
double odometry::getXPos(){ return xPos; }
double odometry::getYPos(){ return yPos; }

int odometry::updatePosition(){
  double loopTime;
  double deltaL;
  double deltaR;
  double deltaS;
  double prevLeftEnc = 0;
  double prevRightEnc = 0;
  double prevBackEnc = 0;
  double currLeftEnc = 0;
  double currRightEnc = 0;
  double currBackEnc = 0;
  

  // This loop will update the x and y position and angle the bot is facing
  while(true){

    //Current time at start of loop
    loopTime = pros::millis();

    currLeftEnc = robot1->leftTracker.get_position()/100.0;
    currRightEnc = robot1->rightTracker.get_position()/100.0;
    currBackEnc = robot1->backTracker.get_position()/100.0;

    //The change in encoder values since last cycle in inches
    deltaL = (currLeftEnc - prevLeftEnc) * robot1->getWheelCir()/360;
    deltaR = (currRightEnc - prevRightEnc)* robot1->getWheelCir()/360;
    deltaS = (currBackEnc - prevBackEnc)* robot1->getWheelCir()/360;

    //Update previous value of the encoders
    prevLeftEnc = currLeftEnc;
    prevRightEnc = currRightEnc;
    prevBackEnc = currBackEnc;

    double h;
    double i;
    double h2;

   
    double a = (deltaL - deltaR)/(robot1->getsL() + robot1->getsR());
    //double a = (angleD - robot1->gyroM.rotation(deg)) * (robot1->getPI()/180);

    if(a){
      double r = deltaR/a;
      i = a / 2.0;
      double sinI = sin(i);
      h = (r + robot1->getsR()) * sinI * 2.0;

      double r2 = deltaS/a;
      h2 = (r2 + robot1->getsS()) * sinI * 2.0;
    } else {
      h = deltaR;
      i = 0;
      h2 = deltaS;
    }
    double p = i + angleR;
    double cosP = cos(p);
    double sinP = sin(p);

    yPos += h*cosP;
    xPos += h*sinP;

    yPos += h2*(-sinP);
    xPos += h2*cosP;
    angleR += a;
    angleD = angleR * (180/robot1->getPI());
    //Delays task so it does not hog all resources
    pros::Task::delay(15 - (pros::millis() - loopTime));
  }
  return 1;
}

int odometry::updateScreen(){
    // Clears controller the screen.
  robot1->mController.clear();
  double loopTime;
  while(true){
    loopTime = pros::millis();
    // Prints the x and y coordinates and angle the bot is facing to the Controller.
    robot1->mController.print(0, 0, "x: %.1fin y: %.1fin     ", -xPos, -yPos);
    //robot1->mController.print(0, 0, "G: %.0lf, Prox: %d   ", rgb_value.green, robot1->colorSensor1.get_proximity());
    pros::Task::delay(50);
    robot1->mController.print(1, 0, "Angle: %.1f°    ", robot1->gyroM.get_heading());
    //robot1->mController.print(1, 0, "R: %.0lf , B: %.0lf     ", rgb_value.red, rgb_value.blue);
    pros::Task::delay(50);

    //robot1->mController.print(2, 0, "B: %.0lf, Prox: %d   ", rgb_value.blue, robot1->colorSensor.get_proximity());
    pros::Task::delay(50);

    // Prints information about the bot to the console
    //printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, robot1->gyroM.get_heading());
    printf("rightTW: %.0lf, leftTW: %0.lf, backTW: %.0lf\n", robot1->rightTracker.get_position()*100.0, robot1->leftTracker.get_position()*100.0, robot1->backTracker.get_position()*100.0);
    //printf("%.0lf, %.0lf, %.0lf \n", Brain.Timer.time(msec), flyOuttake.velocity(rpm), flyOuttake.voltage(voltageUnits::mV));

    //Delays task so it does not hog all resources
    pros::Task::delay(200 - (pros::millis()-loopTime));
  }

  return 1;
}