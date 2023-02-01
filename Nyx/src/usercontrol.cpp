#include "usercontrol.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#define TURN_SENSITIVITY 1.0
#define MOVE_SENSITIVITY 1.0
userControl::userControl(robotChasis *robot){
  robot1 = robot;
  robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}

void userControl::setBrakeMode(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_HOLD);
  else if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_COAST);
}


void userControl::tank() {
  // Put the joysticks through the curve function
  int l_stick = robot1->mController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int r_stick = robot1->mController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  // Set robot to l_stick and r_stick, check joystick threshold
  joy_thresh_opcontrol(l_stick, r_stick);
}

void userControl::set_joystick_threshold(int threshold) { JOYSTICK_THRESHOLD = abs(threshold); }

void userControl::joy_thresh_opcontrol(int l_stick, int r_stick) {
  // Threshold if joysticks don't come back to perfect 0
  if (abs(l_stick) > JOYSTICK_THRESHOLD || abs(r_stick) > JOYSTICK_THRESHOLD) {
    set_tank(l_stick, r_stick);
  }
  else {
    set_tank(0,0);
  }
}

void userControl::set_tank(int left, int right) {
  if (pros::millis() < 1500) return;

  robot1->leftFront.move_voltage(left * (12000.0 / 127.0));
  robot1->leftMid.move_voltage(left * (12000.0 / 127.0));
  robot1->leftRear.move_voltage(left * (12000.0 / 127.0));
  robot1->rightFront.move_voltage(right * (12000.0 / 127.0));
  robot1->rightMid.move_voltage(right * (12000.0 / 127.0));
  robot1->rightRear.move_voltage(right * (12000.0 / 127.0));
  //robot1->mController.print(0, 0, "left: %.1f", left* (12000.0 / 127.0));
}

void userControl::flyControl(){
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
    fly = true;
  }
    if(fly){
      robot1->fly1.move_voltage(11200);
      robot1->fly2.move_voltage(11200);
    }else {
      robot1->fly1.move_voltage(0);
      robot1->fly2.move_voltage(0);
    }
}

void userControl::indexing(){
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
    pros::delay(100);
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
    pros::delay(100);
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
  }
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
    robot1->indexer.set_value(true);
    pros::delay(300);
    robot1->indexer.set_value(false);
  }
}

void userControl::angler(){
    if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
    robot1->angler.set_value(true);
    pros::delay(1000);
    robot1->angler.set_value(false);
  }
}

void userControl::driveLoop(){
  set_joystick_threshold(5);
  fly = false;
  while(true){
    tank();
    flyControl();
    indexing();
    angler();
    pros::Task::delay(20);
  }
}