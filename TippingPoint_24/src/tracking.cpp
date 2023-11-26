#include "tracking.h"


odometry::odometry(robotChasis *robot, double x){
  robot1 = robot;
  yPos = x;
}

double odometry::getXPos(){ return yPos*-1; }

int odometry::updatePosition(){
  double loopTime;
  double currLeftEnc = 0;
  double currRightEnc = 0;

  // This loop will update the x and y position and angle the bot is facing
  while(true){
    
    //Current time at start of loop
    loopTime = robot1->Brain.Timer.time(msec);

    currLeftEnc = robot1->leftTracker.position(deg);
    currRightEnc = robot1->rightTracker.position(deg);

    leftPos = (currLeftEnc/360) * robot1->getWheelCir();
    rightPos = (currRightEnc/360) * robot1->getWheelCir();

    yPos = (leftPos+rightPos)/2;

    //Delays task so it does not hog all resources
    task::sleep(10 - (robot1->Brain.Timer.time(msec)-loopTime));
  }
  return 1;
}

int odometry::updateScreen(){
  // Clears controller the screen.
  robot1->Controller1.Screen.clearScreen();
  double loopTime;

  while(true){
    loopTime = robot1->Brain.Timer.time(msec);

    // Prints the x and y coordinates and angle the bot is facing to the Controller.
    robot1->Controller1.Screen.setCursor(0, 0);
    robot1->Controller1.Screen.print("y: %.1fin     ", yPos);
    robot1->Controller1.Screen.newLine();
    robot1->Controller1.Screen.print("Angle: %.1f°    ", robot1->gyroM.angle());
    //robot1->Controller1.Screen.print("roll: %.1f     ", robot1->gyroM.pitch());
    robot1->Controller1.Screen.newLine();
    //robot1->Controller1.Screen.print("right: %.1lf  ", robot1->rightTracker.position(deg));
    
    //printf("right: %lf  ", robot1->rightTracker.position(deg));
    //printf("left: %lf    ", robot1->leftTracker.position(deg));
    //printf("back: %lf     \n", robot1->backTracker.position(deg));
    
    
    //robot1->Controller1.Screen.print("left: %.1lf     ", robot1->leftTracker.position(deg));
    //robot1->Controller1.Screen.newLine();
    //robot1->Controller1.Screen.print("back: %.1lf     ", robot1->backTracker.position(deg));
    // Controller1.Screen.print("Drive mV: %.0lf");

    //Prints information about the bot to the console
    //printf("Distance: %.2lf Y Voltage: %.0f X Voltage: %.0f\n", vMag, yVoltage, xVoltage);
    //printf("Tracking Wheels Angle: %0.f   IMU angle: %0.lf\n", angleD, robot1->gyroM.rotation(deg));
    //printf("rightTW: %.0lf, leftTW: %0.lf, backTW: %.0lf\n", robot1->rightTracker.position(deg), robot1->leftTracker.position(deg), robot1->backTracker.position(deg));
    //printf("Flywheel RPM: %.1lf, Flywheel Voltage: %.0lf\n\n\n", robot1->flyOuttake.velocity(rpm), robot1->flyOuttake.voltage(voltageUnits::mV));
    //printf("%.0lf, %.0lf, %.0lf \n", Brain.Timer.time(msec), flyOuttake.velocity(rpm), flyOuttake.voltage(voltageUnits::mV));

    //Delays task so it does not hog all resources
    task::sleep(200 - (robot1->Brain.Timer.time(msec)-loopTime));
  }

  return 1;
}