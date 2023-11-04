/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FL                   motor         18              
// FR                   motor         6               
// ML                   motor         20              
// MR                   motor         10              
// BL                   motor         16              
// BR                   motor         19              
// Cata                 motor         14              
// Intake               motor         1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "iostream"

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  FL.setPosition(0, degrees);
  ML.setPosition(0, degrees);
  BL.setPosition(0, degrees);
  FR.setPosition(0, degrees);
  MR.setPosition(0, degrees);
  BR.setPosition(0, degrees);
  Intake.setPosition(0, degrees);
  Cata.setPosition(0, degrees);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//  --- General Fields ---
double pi = 3.14159265;
double degToInch = (1/360) * pi * 3.25 * (3/4);
bool resetDriveEncoders = false;

//  --- PID Fields ---
bool enableDrivePID = true;

// Settings
double kP = 0.1;
double kI = 0.001;
double kD = 0.001;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Autonomous Settings
int desiredValue = 5;
int desiredTurnValue = 0;

int error; // SensorVal - TargetVal : Positional Value
int prevError = 0; // Position 20 ms ago
int derivative; // error - prevError : Speed
int totalError = 0; // Integral : Absement
int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError = 0;

int drivePID() {

  while(enableDrivePID) {
    if(resetDriveEncoders) {
      resetDriveEncoders = false;

      FL.setPosition(0, degrees);
      ML.setPosition(0, degrees);
      BL.setPosition(0, degrees);
      FR.setPosition(0, degrees);
      MR.setPosition(0, degrees);
      BR.setPosition(0, degrees);
    }
    // -------------------------- Lateral Movement PID -------------------------- //

    int leftMotorPositionDeg = ML.position(degrees);
    int rightMotorPositionDeg = MR.position(degrees);
    int averagePosition = (leftMotorPositionDeg + rightMotorPositionDeg) / 2;
    // int averagePosition = averagePositionDeg / degToInch;

    error = desiredValue - averagePosition;
    derivative = error - prevError;
    totalError += error;

    double latMotorPower = ((error * kP) + (totalError * kI) + (derivative * kD)); 
    cout << latMotorPower << endl;
    // -------------------------- Lateral Movement PID -------------------------- //

    // -------------------------- Turning Movement PID -------------------------- //
    // int turnDifference = leftMotorPositionDeg - rightMotorPositionDeg;

    // turnError = turnDifference - desiredTurnValue;
    // turnDerivative = turnError - turnPrevError;
    // turnTotalError += turnError;

    // double turnMotorPower = ((turnError * turnkP) + (turnTotalError * turnkI) + (turnDerivative * turnkD)) / 12.0; 

    // -------------------------- Turning Movement PID -------------------------- //
    FL.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);
    BL.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);
    FR.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);
    MR.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);
    BR.spin(vex::directionType::fwd, latMotorPower, vex::velocityUnits::pct);

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }

  return 1;
}

void autonomous(void) {
  vex::task runDrivePID(drivePID);
  resetDriveEncoders = true;
  desiredValue = 235;
  vex::task::sleep(1000);
  resetDriveEncoders = true;
  desiredTurnValue = 300;

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  double driveConstant = 1;
  while (1) {
    int vertical = Controller1.Axis3.value();
    int horizontal = Controller1.Axis4.value();
    bool L1 = Controller1.ButtonL1.pressing();
    bool L2 = Controller1.ButtonL2.pressing();
    bool R2 = Controller1.ButtonR2.pressing();
    bool R1 = Controller1.ButtonR1.pressing();
    int stickDeadZone = 5;

    double turnImportance = 0.5;
    horizontal *= turnImportance;

    if(vertical > 0) {
      vertical = (vertical * vertical) / 200;
    } else if(vertical < 0) {
      vertical = (vertical * vertical) / -200;
    }

    if(horizontal > 0) {
      horizontal = (horizontal * horizontal) / 200;
    } else if(vertical < 0) {
      horizontal = (horizontal * horizontal) / -200;
    }

    // Left wheel control
    if(abs(vertical) > stickDeadZone || abs(horizontal) > stickDeadZone) { // Check dead zone
      FL.spin(vex::directionType::fwd, (vertical + horizontal) * driveConstant, vex::velocityUnits::pct);
      ML.spin(vex::directionType::fwd, (vertical + horizontal) * driveConstant, vex::velocityUnits::pct);
      BL.spin(vex::directionType::fwd, (vertical + horizontal) * driveConstant, vex::velocityUnits::pct);
      FR.spin(vex::directionType::fwd, (vertical - horizontal) * driveConstant, vex::velocityUnits::pct);
      MR.spin(vex::directionType::fwd, (vertical - horizontal) * driveConstant, vex::velocityUnits::pct);
      BR.spin(vex::directionType::fwd, (vertical - horizontal) * driveConstant, vex::velocityUnits::pct);
    }
    else {
      FL.stop(coast);
      ML.stop(coast);
      BL.stop(coast);
      FR.stop(coast);
      MR.stop(coast);
      BR.stop(coast);
    }

    // // Right wheel control
    // if(abs(horizontal) > stickDeadZone) {
    //   FL.spin(vex::directionType::fwd, horizontal, vex::velocityUnits::pct);
    //   ML.spin(vex::directionType::fwd, horizontal, vex::velocityUnits::pct);
    //   BL.spin(vex::directionType::fwd, horizontal, vex::velocityUnits::pct);
    //   FR.spin(vex::directionType::fwd, -horizontal, vex::velocityUnits::pct);
    //   MR.spin(vex::directionType::fwd, -horizontal, vex::velocityUnits::pct);
    //   BR.spin(vex::directionType::fwd, -horizontal, vex::velocityUnits::pct);
    
    // }
    // else {
    //   FL.stop(coast);
    //   ML.stop(coast);
    //   BL.stop(coast);
    //   FR.stop(coast);
    //   MR.stop(coast);
    //   BR.stop(coast);
    // }

    if(!L1 && !R1) {
      Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if(L1 && !R1) {
      Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    }

    // if(Cata.position(degrees) < 360 || Cata.position(degrees) > 360) {
    //   cataFire = false;
    //   Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    // }
    // else if(R2 || cataFire) {
    //   cataFire = true;
    //   Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    // }

    if(R1) {
       Cata.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
       Intake.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
    } else {
      Cata.stop();
    }

    wait(20, msec); // Sleep the task for a short amount of time to
            
  }                // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}