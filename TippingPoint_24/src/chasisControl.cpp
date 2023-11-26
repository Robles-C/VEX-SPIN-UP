#include "chasisControl.h"

autonomousControl::autonomousControl(robotChasis *robot, odometry *tr){
  robot1 = robot;
  tracking = tr;
}

void autonomousControl::setPIDConstants(float xkP, float xkI, float xkD, int xCap,
                                        float turnkP, float turnkI, float turnkD, int turnCap,
                                        float tiltkP, float tiltkI, float tiltkD, int tiltCap){
  xPID.kP = xkP; xPID.kI = xkI; xPID.kD = xkD; xPID.cap = xCap;
  turnPID.kP = turnkP; turnPID.kI = turnkI; turnPID.kD = turnkD; turnPID.cap = turnCap;
  tiltPID.kP = tiltkP; tiltPID.kI = tiltkI; tiltPID.kD = tiltkD; tiltPID.cap = turnCap;                  
}

void autonomousControl::moveDrive(float vX, float vTurn){ 
  robot1->LeftFront.spin(fwd,vX*leftMult + vTurn , voltageUnits::mV); 
  robot1->LeftMid.spin(fwd,vX*leftMult + vTurn, voltageUnits::mV);
  robot1->LeftRear.spin(fwd,vX*leftMult + vTurn , voltageUnits::mV);
  robot1->RightFront.spin(fwd,vX*rightMult - vTurn, voltageUnits::mV);
  robot1->RightMid.spin(fwd,vX*rightMult - vTurn,voltageUnits::mV);
  robot1->RightRear.spin(fwd,vX*rightMult - vTurn, voltageUnits::mV);
}

float autonomousControl::averageRPM(){
  return (fabs(robot1->RightFront.velocity(rpm)) + fabs(robot1->LeftFront.velocity(rpm)) + fabs(robot1->RightMid.velocity(rpm)) + fabs(robot1->LeftMid.velocity(rpm))+fabs(robot1->RightRear.velocity(rpm)) + fabs(robot1->LeftRear.velocity(rpm)))/3;
}

float autonomousControl::updatePID(PIDSettings *good){
  good->error = good->curr - good->target; 
  good->derivative = good->error - good->prevError;
  good->totalError = good->totalError + good->error;
  //good-error = E(s) good-curr = Y*(s) good-target(Y(s) prevError = -e(t) totalError = H(s) 

  if((good->totalError*good->kI)>good->cap) good->totalError = good->cap/good->kI;
  else if((good->totalError*good->kI)<-good->cap)good->totalError = -good->cap/good->kI;

  if(std::signbit(good->error) != std::signbit(good->prevError)) good->totalError = 0;

  good->prevError = good->error;
  return -(good->kP*good->error + good->kD*good->derivative + good->kI*good->totalError);
}

void autonomousControl::movAB(){
  updateCurrPos(); 

  float turnVoltage, xVoltage;

  switch(mState) {
  case 1:
  if(transPosE && canShift()){
      robot1->set_drive_break_type(coast);
      wait(15, msec);
      robot1->trans.open();

    }
    
    if(rushE == true){
      if(canShift()){
        wait(200, msec);
        robot1->raise.open();
      }
      if(robot1->leftTouch.pressing()){
        wait(10, msec);
        robot1->leftHand.close();
        leftGrab = true;
      }
      if (robot1->rightTouch.pressing()){
        wait(15, msec);
        robot1->rightHand.close();
        rightGrab = true;
      }
      if(leftGrab && rightGrab == true){
        wait(100, msec);
        robot1->raise.close();
    }
    }
    //move in x direction
    xVoltage = updatePID(&xPID);
    turnVoltage = 0;
    if(fabs(turnPID.curr - turnPID.target)>6){
      if(turnPID.curr<0){
        rightMult = .8;
      }else if(turnPID.curr > 0){
        leftMult = .8;
      }else{
        leftMult = 1;
        rightMult = 1;
      }
    }
    moveDrive(xVoltage, turnVoltage);
      if(((fabs(xPID.curr - xPID.target) < 1) && !isMovingD()) || (rightGrab && leftGrab)){
        xPID.curr = 0;
        xPID.target = 0;
        if(transPosE){
          //manualShift();
          transPosE = false;
        }
        mState = 2;
      }
    break;
  case 2:
  //turn to final position
    turnVoltage = updatePID(&turnPID);
    xVoltage = 0;
    moveDrive(xVoltage, turnVoltage);
    
    if((fabs(turnPID.curr - turnPID.target) < 5) && !isMovingD()){
      mState = 0;
      turnPID.curr = 0;
      turnPID.target = 0;
      xPID.curr = 0;
      posUpdated = false;
    }
    break; 

  default:
    xVoltage = 0;
    turnVoltage = 0;
    //tiltVoltage = updatePID(&tiltPID);
    moveDrive(xVoltage,  turnVoltage);
    if(posUpdated){
      mState = 1;
    }
    break;
}
//Printing different values to the console for debugging purposes
  //printf(" mState: %d, pidTarget: %f, angleD: %f\n", mState, turnPID.target, tracking->getangleD());
 printf("%f   %f ", turnPID.target, turnPID.curr);
 //printf("%f ", robot1->gyroM.pitch());
  printf("%f   %f  %d \n", xPID.target, xPID.curr, mState);
  
  //printf("%f   %f  %d\n", turnPID.target, turnPID.curr, mState);
}

bool autonomousControl::isMovingD(){
  //rightSide and leftSide are variables that represent the rotations per minute of each side
  float rightSide, leftSide;
  //rightSide = average of (rightMid motor + rightFront motor + rightBack motor)
  rightSide = (robot1->RightMid.velocity(rpm) + robot1->RightFront.velocity(rpm)+robot1->RightRear.velocity(rpm))/3;
  //leftSide = average of (leftMid motor + leftFront motor + leftBack motor)
  leftSide = (robot1->LeftMid.velocity(rpm) + robot1->LeftFront.velocity(rpm)+robot1->LeftRear.velocity(rpm))/3;
  
  
  //if no longer moving - right rpm + left rpm < 1
  // dont set 0 as the threshhold for rpm because robot may never reach 0 rpm and it will always be consideered "moving"
  if((fabs(rightSide) + fabs(leftSide)) < 1){
    // return false - not moving rpm is less than 1
    return false;
  }
  //return true - still moving rpm is greater than 1
  return true;
}

bool autonomousControl::canShift(){
  //rightSide and leftSide are variables that represent the rotations per minute of each side
  float rightSide, leftSide;
  //rightSide = average of (rightMid motor + rightFront motor + rightBack motor)
  rightSide = (robot1->RightMid.velocity(rpm) + robot1->RightFront.velocity(rpm)+robot1->RightRear.velocity(rpm))/3;
  //leftSide = average of (leftMid motor + leftFront motor + leftBack motor)
  leftSide = (robot1->LeftMid.velocity(rpm) + robot1->LeftFront.velocity(rpm)+robot1->LeftRear.velocity(rpm))/3;
  
  
  //if no longer moving - right rpm + left rpm < 1
  // dont set 0 as the threshhold for rpm because robot may never reach 0 rpm and it will always be consideered "moving"
  if((fabs(rightSide) + fabs(leftSide)) < 50){
    // return false - not moving rpm is less than 1
    return false;
  }
  //return true - still moving rpm is greater than 1
  return true;
}

void autonomousControl::updateTargetPos(float x, float finalOrientation, bool transPos, bool rush){
  turnPID.target = finalOrientation;
  xPID.target = x;
  tiltPID.target=0;
  transPosE = transPos;
  rushE = rush;
  posUpdated = true;
}

void autonomousControl::updateIntakePct(int pow){}


void autonomousControl::intakeMove(){
  //robot1->leftIntake.spin(fwd, intakePct, pct);
  //robot1->rightIntake.spin(fwd, -intakePct, pct);
}


void autonomousControl::waitUntilSettled(){
  wait(100, msec);
  while(averageRPM() >= .5){
    wait(20, msec);
  }
}



void autonomousControl::waitUntilDeg(float deg){
  wait(100, msec);
  while(deg < fabs(turnPID.curr - turnPID.target) ){
    wait(20, msec);
  }
}

void autonomousControl::strafeVision(){}


void autonomousControl::updateCurrPos(){
  if(robot1->gyroM.angle()>250){
    turnPID.curr = robot1->gyroM.angle()-360;
  }else{
    turnPID.curr = robot1->gyroM.angle();
  }
  xPID.curr = tracking->getXPos();
  tiltPID.curr = robot1->gyroM.pitch();
}

void autonomousControl::updateRaise(bool tr){
  if(tr){
    robot1->trans.open();
  }else{
    robot1->trans.close();
  }
}

void autonomousControl::updateLeftHand(bool lh){
  if(lh){
    robot1->trans.open();
  }else{
    robot1->trans.close();
  }
}

void autonomousControl::updateRightHand(bool rh){
  if(rh){
    robot1->trans.open();
  }else{
    robot1->trans.close();
  }
}

void autonomousControl::autoMain(){
  robot1->set_drive_break_type(brake);
  /*robot1->rightHand.close();
  robot1->leftHand.close();
  robot1->frontHand.open();
  robot1->raise.close();
  robot1->trans.open();*/
  while(true){
    movAB();
    task::sleep(20);
  }
}

void autonomousControl::manualShift(){
    robot1->RightFront.rotateFor(forward, 400, msec);
    robot1->RightMid.rotateFor(forward, 400, msec);
    robot1->RightRear.rotateFor(forward, 400, msec);
    robot1->LeftFront.rotateFor(forward, 400, msec);
    robot1->LeftMid.rotateFor(forward, 400, msec);
    robot1->LeftRear.rotateFor(forward, 400, msec);
    robot1->trans.close();
}