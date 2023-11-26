#include "usercontrol.h"
#include "visionUno.h"

userControl::userControl(robotChasis *robot, bool dM) {
  robot1 = robot;
  robot1->set_drive_break_type(coast);
}

void userControl::setBrakeMode() {
  /*if (robot1->Controller1.ButtonB.pressing())
    robot1->set_drive_break_type(hold);
  else if (robot1->Controller1.ButtonA.pressing())
    robot1->set_drive_break_type(coast);
*/}

void userControl::driveB() {
  robot1->LeftRear.spin(fwd,robot1->Controller1.Axis3.value(), velocityUnits::pct);
  robot1->LeftFront.spin(fwd, (robot1->Controller1.Axis3.value()), velocityUnits::pct); 
  robot1->LeftMid.spin(fwd, (robot1->Controller1.Axis3.value()), velocityUnits::pct);
  robot1->RightRear.spin(fwd, robot1->Controller1.Axis2.value(), velocityUnits::pct);
  robot1->RightFront.spin(fwd, (robot1->Controller1.Axis2.value()), velocityUnits::pct);
  robot1->RightMid.spin(fwd, (robot1->Controller1.Axis2.value()),velocityUnits::pct);
}

void userControl::turnUntil() {
//leave blank I wanna try something here-cristian
}


void userControl::driveLoop() {
  while (true) {
    setBrakeMode();
    driveB();
    //shift
  
    if(robot1->Controller1.ButtonLeft.pressing()){
        robot1->raiseFront.spin(fwd, 100,velocityUnits::pct);
    }else if(robot1->Controller1.ButtonRight.pressing()){
        robot1->raiseFront.spin(reverse, 100,velocityUnits::pct);
    }else{
        robot1->raiseFront.spin(reverse, 0,velocityUnits::pct);
        robot1->raiseFront.setBrake(brake);
    }
    //trans
    if(robot1->Controller1.ButtonUp.pressing()){
      robot1->trans.open();
      robot1->LeftRear.spin(reverse,100, velocityUnits::pct);
      robot1->LeftFront.spin(reverse, 100, velocityUnits::pct); 
      robot1->LeftMid.spin(reverse, 100, velocityUnits::pct);
      robot1->RightRear.spin(reverse, 100, velocityUnits::pct);
      robot1->RightFront.spin(reverse, 100, velocityUnits::pct);
      robot1->RightMid.spin(reverse, 100,velocityUnits::pct);
      robot1->set_drive_break_type(coast);
    }else if(robot1->Controller1.ButtonDown.pressing()){
     robot1->trans.close();
      robot1->LeftRear.spin(reverse,100, velocityUnits::pct);
      robot1->LeftFront.spin(reverse, 100, velocityUnits::pct); 
      robot1->LeftMid.spin(reverse, 100, velocityUnits::pct);
      robot1->RightRear.spin(reverse, 100, velocityUnits::pct);
      robot1->RightFront.spin(reverse, 100, velocityUnits::pct);
      robot1->RightMid.spin(reverse, 100,velocityUnits::pct);
      robot1->set_drive_break_type(hold);
    }
    //left hand
    if(robot1->Controller1.ButtonL1.pressing()){
      robot1->leftHand.close();
    }else if(robot1->Controller1.ButtonL2.pressing()){
      robot1->leftHand.open();
    }
    //right hand
    if(robot1->Controller1.ButtonR1.pressing()){
      robot1->rightHand.close();
    }else if(robot1->Controller1.ButtonR2.pressing()){
      robot1->rightHand.open();
    }
    //front hand
    if(robot1->Controller1.ButtonY.pressing()){
      robot1->frontHand.close();
    }else if(robot1->Controller1.ButtonA.pressing()){
      robot1->frontHand.open();
    }
    //raise
    if(robot1->Controller1.ButtonX.pressing()){
      robot1->raise.close();
    }else if(robot1->Controller1.ButtonB.pressing()){
      robot1->raise.open();
    }
    wait(20, msec);
  }
}