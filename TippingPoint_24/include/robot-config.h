#pragma once
#include "v5.h"
#include "v5_vcs.h"
using namespace vex;

/// Class for robot configuration.
///
/// A class for storing motor, sensor, brain, controller objects.
/// This class also holds most constants used throught the program.
/// Classes like \ref odometry, autonomousControl, and userControl will use
/// this class to access sensor data and motors.
class robotChasis{
  public:
    /// Gets pi value.
    ///
    /// PI value is derived from the pi approximation 355/113.
    /// @return pi value as a double
    double getPI();

    /// Gets left tracking wheel position.
    ///
    /// Current left tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sL as a float.
    float getsL();

    /// Gets right tracking wheel position.
    ///
    /// Current right tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sR as a float.
    float getsR();

    /// Gets back tracking wheel position.
    ///
    /// Current back tracking wheel position in degree units will be aqquired using this method.
    /// @return value of sS as a float.
    float getsS();

    /// Gets wheel circumference value of the tracking wheels.
    ///
    /// Wheel circumference of the right, left, back tracking wheels.
    /// @return value wheelCir as a double
    double getWheelCir();
    
    brain Brain;

    competition Comp;

    controller Controller1;

    // Declare Drive Motor Objects Here
    motor LeftFront = motor(PORT7, ratio18_1, true);
    motor LeftMid = motor(PORT9, ratio18_1, false);
    motor LeftRear = motor(PORT10, ratio18_1, true);
    motor RightFront = motor(PORT1, ratio18_1, false);
    motor RightMid = motor(PORT2, ratio18_1, true);
    motor RightRear = motor(PORT3, ratio18_1, false);
    motor raiseFront = motor(PORT11, ratio18_1, true);
    motor frontArm = motor(PORT8, ratio36_1, true);


    // Declare Any pneumatics Here
    pneumatics trans = pneumatics(Brain.ThreeWirePort.A);
    pneumatics raise = pneumatics(Brain.ThreeWirePort.G);
    pneumatics leftHand = pneumatics(Brain.ThreeWirePort.E);
    pneumatics rightHand = pneumatics(Brain.ThreeWirePort.D);
    pneumatics frontHand = pneumatics(Brain.ThreeWirePort.H);

    // Declare odometry encoders/rotation sensors here
    rotation leftTracker = rotation(PORT20, true);
    rotation rightTracker = rotation(PORT15, false);
    // Declare any other sensors here
    limit leftTouch = limit(Brain.ThreeWirePort.B);
    limit rightTouch = limit(Brain.ThreeWirePort.C);
    limit frontTouch = limit(Brain.ThreeWirePort.F);
    inertial gyroM = inertial(PORT11);
    

    /// Sets the brake type for the drive motors.
    ///
    /// Will set each drive motor to the brake type provided in the parameter field.
    /// @param B the type of break you want your to set your motors to.
    /// \see <a href="https://api.vexcode.cloud/v5/html/namespacevex.html#a2090f7d57d63c5cf693f269bc73568f1">vex::brakeType</a>
    void set_drive_break_type(brakeType B);

    /// Stops all drive motors.
    ///
    /// Will brake all drive motors according their current brakeType.
    /// \see <a href="https://api.vexcode.cloud/v5/html/namespacevex.html#a2090f7d57d63c5cf693f269bc73568f1">vex::brakeType</a>
    void stopMotors();

    /// Constructor for the class robotChasis.
    ///
    /// This constructor will create an instance of the class robotChasis.
    /// It will set values to wheelDiameter, sL, sR, and sS.
    /// @param wD Wheel diameter in inches.
    /// @param tcL Left Tracking Wheel distance to tracking center. 
    /// @param tcR Right Tracking Wheel distance to tracking center. 
    /// @param tcB Back Tracking Wheel distance to tracking center. 
    robotChasis(float wD);

  private:

    float wheelDiameter;
    double wheelCir;
    const double PI = 22/7;

    float sL;
    float sR;
    float sS;

    int autonSelect;
};

void  vexcodeInit( robotChasis *robot1 );