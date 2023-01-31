#include "usercontrol.h"
#define TURN_SENSITIVITY 1.0
#define MOVE_SENSITIVITY 1.0
userControl::userControl(robotChasis *robot){
  robot1 = robot;
  robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::storageRoller(){
}

void userControl::setBrakeMode(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_HOLD);
  else if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::intakeM(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
    robot1->leftIntake.move(127);
    robot1->rightIntake.move(127);
  } else if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    robot1->leftIntake.move(-127);
    robot1->rightIntake.move(-127);
  } else {
    robot1->leftIntake.move(0);
    robot1->rightIntake.move(0);
  }
}

void userControl::liftControl(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
    robot1->roller1.move(127);
    robot1->roller2.move(127);
  } else if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
    robot1->roller1.move(-127);
    robot1->roller2.move(-127);
  } else {
    robot1->roller1.move(0);
    robot1->roller2.move(0);
  }
}

void userControl::driveM(){
  a3 = robot1->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  a4 = robot1->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
  a1 = robot1->mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  if((std::abs(a3) > 10) || (std::abs(a4) > 10) || (std::abs(a1) > 10)){
    robot1->frontRight.move((a3 * MOVE_SENSITIVITY) - (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    robot1->frontLeft.move(-(a3 * MOVE_SENSITIVITY) - (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    robot1->backRight.move((a3 * MOVE_SENSITIVITY) + (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
    robot1->backLeft.move(-(a3 * MOVE_SENSITIVITY) + (a4 * MOVE_SENSITIVITY) - (a1 * TURN_SENSITIVITY));
  } else {
    robot1->frontRight.move(0);
    robot1->frontLeft.move(0);
    robot1->backRight.move(0);
    robot1->backLeft.move(0);
  }
}

void userControl::driveLoop(){
  while(true){
    intakeM();
    setBrakeMode();
    liftControl();
    driveM();

    pros::Task::delay(20);
  }
}