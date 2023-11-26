#pragma once
#include "chasisControl.h"
#include "robot-config.h"

class autonomousRoutine {
  public:
    void run(int autoSelection);
    autonomousRoutine(autonomousControl *autoControl);
    bool isMov = false;
    

  private:
    autonomousControl *control;
    void odometryOnlyAuto();
    void test();
    void odometryVisionAuto();
};