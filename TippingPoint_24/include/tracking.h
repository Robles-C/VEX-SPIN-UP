#pragma once
#include "robot-config.h"
#include <math.h>
#include <stdio.h>

class odometry{
public:
  int updatePosition();
  int updateScreen();
  double getXPos();
  double getYPos();
  double getangleR();
  double getangleD();
  odometry(robotChasis *robot, double x);

private:
  double leftPos;
  double rightPos;
  double angleR;
  double angleD;
  double yPos;
  double currAngle;
  robotChasis *robot1;
};