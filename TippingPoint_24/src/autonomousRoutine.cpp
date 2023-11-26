#include "autonomousRoutine.h"

autonomousRoutine::autonomousRoutine(autonomousControl *autoControl) {
  control = autoControl;
  
  control->setPIDConstants( 2500, 0, 11000, 0,
                            200, 0, 110, 0,
                            1200, 0, 2500, 0);
                            //PID constants
                            //kP,kI,kD, driveCap
                            //kP,kI,kD, turnCap
}

void autonomousRoutine::run(int autoSelection) {
  switch(autoSelection){
    // pass in one of the cases in the main in order to select one of the autons to run
    case 0:
      test();
      break;
    case 1:
      odometryOnlyAuto();
      break;
    case 2:
      odometryVisionAuto();
      break;
    default:
      printf("No Auto Selected!");
      break;
  }
}

void autonomousRoutine::test(){

}

void autonomousRoutine::odometryOnlyAuto(){
  //linear distance, final angle, transmission, rush
  control->updateLeftHand(false);
  control->updateRightHand(false);
  control->updateTargetPos(-48, 0,true,true);
  control->updateTargetPos(24, -90, false, false);
  control->updateTargetPos(-48,0,false,false);
}

void autonomousRoutine::odometryVisionAuto(){
  //autonomous paths using odometry + vision
}