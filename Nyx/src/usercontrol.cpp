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

void userControl::expansionControl(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_UP)&&expan >3) {
    robot1->set_drive_break_type(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(150);
    expan--;
  }
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_UP)&&expan >0){
    expan--;
    pros::delay(150);
  }
  if(expan==0){
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    robot1->expansion.set_value(true);
    pros::delay(200);
    robot1->expansion.set_value(false);
    pros::delay(200);
    expan+=5;
  }
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


void userControl::indexing(){
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
    
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
    pros::delay(250);
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
    pros::delay(250);
    robot1->indexer.set_value(true);
    pros::delay(250);
    robot1->indexer.set_value(false);
  }
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
    
    robot1->indexer.set_value(false);
    pros::delay(270);
    robot1->indexer.set_value(true);
  }
}

void userControl::flyControl(){
  if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    robot1->angler.set_value(true);
    robot1->fly1.move_voltage(0);
    robot1->fly2.move_voltage(0);
  }else if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
    robot1->angler.set_value(true);
    robot1->fly1.move_voltage(8800);
    robot1->fly2.move_voltage(8800);
  }else if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
    robot1->angler.set_value(false);
    robot1->fly1.move_voltage(10000);
    robot1->fly2.move_voltage(10000);
  }else if(robot1->mController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
    robot1->angler.set_value(false);
    robot1->fly1.move_voltage(11200);
    robot1->fly2.move_voltage(11200);
  }
}

void userControl::intake(){
  if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
    robot1->topIntake.move_voltage(12000);
    robot1->bottomIntake.move_voltage(12000);
  }else if(robot1->mController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
    robot1->topIntake.move_voltage(-12000);
    robot1->bottomIntake.move_voltage(-12000);
  }else {
    robot1->topIntake.move_voltage(0);
    robot1->bottomIntake.move_voltage(0);
  }
}

void userControl::driveLoop(){
  set_joystick_threshold(5);
  fly = false;
  expan = 5;
  //robot1->indexer.set_value(true);
  while(true){
    tank();
    flyControl();
    indexing();
    intake();
    expansionControl();
    pros::Task::delay(20);
  }
}