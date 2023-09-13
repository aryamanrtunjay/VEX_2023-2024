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
// FL                   motor         1               
// FR                   motor         2               
// ML                   motor         3               
// MR                   motor         4               
// BL                   motor         5               
// BR                   motor         6               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

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
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;

double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

// Autonomous Settings
int desiredValue = 5;
int desiredTurnValue = 90;

int error; // SensorVal - TargetVal : Positional Value
int prevError = 0; // Position 20 ms ago
int derivative; // error - prevError : Speed
int totalError = 0; // Integral : Absement
int turnError; //
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
    int averagePositionDeg = (leftMotorPositionDeg + rightMotorPositionDeg) / 2;
    int averagePosition = averagePositionDeg / degToInch;

    error = averagePosition - desiredValue;
    derivative = error - prevError;
    totalError += error;

    double latMotorPower = ((error * kP) + (totalError * kI) + (derivative * kD)) / 12.0; 

    // -------------------------- Lateral Movement PID -------------------------- //

    // -------------------------- Turning Movement PID -------------------------- //
    int turnDifference = leftMotorPositionDeg - rightMotorPositionDeg;


    turnError = turnDifference - desiredTurnValue;
    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = ((turnError * turnkP) + (turnTotalError * turnkI) + (turnDerivative * turnkD)) / 12.0; 

    // -------------------------- Turning Movement PID -------------------------- //
    FL.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);
    BL.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);
    FR.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);
    MR.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);
    BR.spin(vex::directionType::fwd, latMotorPower + turnMotorPower, vex::velocityUnits::pct);

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);
  }

  return 1;
}

void autonomous(void) {
  vex::task runDrivePID(drivePID);
  resetDriveEncoders = true;
  desiredValue = 5;
  desiredTurnValue = 600;

  vex::task::sleep(1000);
  resetDriveEncoders = true;
  desiredValue = 5;
  desiredTurnValue = 300;

}


void usercontrol(void) {
  enableDrivePID = false;
  // Driver Control Main Loop
  while (1) {
    int leftStick = Controller1.Axis3.value();
    int rightStick = Controller1.Axis2.value();
    int stickDeadZone = 5;

    // Left wheel control
    if(abs(leftStick) > stickDeadZone) { // Check dead zone
      FL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      ML.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      BL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
    }
    else {
      FL.stop(hold);
      ML.stop(hold);
      BL.stop(hold);
    }

    // Right wheel control
    if(abs(rightStick) > stickDeadZone) {
      FR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      MR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      BR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
    }
    else {
      FR.stop(hold);
      MR.stop(hold);
      BR.stop(hold);
    }

    // wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
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
