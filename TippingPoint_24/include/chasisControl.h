#pragma once

#include "robot-config.h"
#include "tracking.h"
#include <cmath>

class autonomousControl{
  public:
    // forward dis, deg target, update trans, updateraise
    void updateTargetPos(float x, float degTarget, bool transPos, bool rush);
    void updateOrientation(float degTarget);
    void updateIntakePct(int pow);
    void updateRaise(bool raisePos);
    void updateRightHand(bool rightPos);
    void updateLeftHand(bool leftPos);

    void waitUntilDistance(float dis);
    void waitUntilSettled();
    void waitUntilDeg(float deg);

    void autoMain();
    void setPIDConstants(float xkP, float xkI, float xkD, int xCap,
                         float turnkP, float turnkI, float turnkD, int turnCap,
                         float tiltkP, float tiltkI, float tiltkD, int tiltCap);

    autonomousControl(robotChasis *robot, odometry *tr);

  private:
    struct PIDSettings {
      //Group of variables. good is a version of it
      float target; float curr; float error; float prevError;
            float derivative; float totalError;
      float kP, kI, kD;
      //curr is curent value
      // kp is current error
      // I adds error over time till infinity
      // change of error
      int cap;
      // limits I 
    };

    PIDSettings xPID;
    PIDSettings turnPID;
    PIDSettings tiltPID;
    
    robotChasis *robot1;
    odometry *tracking;

    double leftMult = 1;
    double rightMult = 1;
    double rightEncoder;
    double angleVoltage;
    bool posUpdated = false;
    bool transPosE = false;
    bool raisePosE = false;
    bool rushE = false;
    bool leftGrab = false;
    bool rightGrab = false;
    int mState = 0;
    

    void moveDrive(float vX, float vTurn);
    void odometryMove(bool oMove);
    void turnVision();
    void forwardVision();
    void strafeVision();
    void updateCurrPos();
    float averageRPM();
    float updatePID(PIDSettings *good);
    int turnCapp(float distanceMag);
    void movAB();
    void intakeMove();
    bool isMovingD();
    bool canShift();
    void manualShift();
};
