/// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         18              
// FR                   motor         6               
// ML                   motor         20              
// MR                   motor         10              
// BL                   motor         16              
// BR                   motor         19              
// Controller1          controller                    
// Cata                 motor         14              
// Intake               motor         13              
// rightWing            digital_out   A               
// leftWing             digital_out   B               
// Inertial             inertial      11              
// Bumper               bumper        H               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         18              
// FR                   motor         6               
// ML                   motor         20              
// MR                   motor         10              
// BL                   motor         16              
// BR                   motor         19              
// Controller1          controller                    
// Cata                 motor         14              
// Intake               motor         13              
// rightWing            digital_out   A               
// leftWing             digital_out   B               
// Inertial             inertial      11              
// Bumper               bumper        C               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         11              
// FR                   motor         12              
// ML                   motor         14              
// MR                   motor         13              
// BL                   motor         5               
// BR                   motor         1               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "iostream"

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
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
double degToInch = (1.0/360) * pi * 3.25 * (3.0/4.0);
bool resetDriveEncoders = false;
double matchTime = 0;

//  --- PID Fields ---
bool enableDrivePID = true;

// Settings
// double kU = 0.30;
// double pU = 0.70;
// double kP = 0.6 * kU;
// double kI = 0.3 * kU / pU;
// double kD = 0.075 * kU / pU;

double kP = 0.15;
double kI = 0.0;
double kD = 15.0;

double turnkP = 25;
double turnkI = 0.00001;
double turnkD = 0.4;

// Autonomous Settings
double desiredValue = 0;
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
    cout << error * degToInch << endl;
    derivative = error - prevError;
    totalError += error;

    double latMotorPower = ((error * kP) + (totalError * kI) + (derivative * kD)); 
    // -------------------------- Lateral Movement PID -------------------------- //

    // -------------------------- Turning Movement PID -------------------------- //
    double h = Inertial.heading(degrees);
    if(h >= 180) h = h - 360;
    // cout << h << endl;
    turnError = h - desiredTurnValue;

    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = ((turnError * turnkP) + (turnTotalError * turnkI) + (turnDerivative * turnkD)) / 12.0; 
    if(desiredTurnValue >= 360) turnMotorPower = 0;
    turnMotorPower = 0;
    // -------------------------- Turning Movement PID -------------------------- //
    double speedLimit = 0.5;
    latMotorPower = 200;
    FL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower) * speedLimit, vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower) * speedLimit, vex::velocityUnits::pct);
    BL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower) * speedLimit, vex::velocityUnits::pct);
    MR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower) * speedLimit, vex::velocityUnits::pct);
    BR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower) * speedLimit, vex::velocityUnits::pct);
    FR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower) * speedLimit, vex::velocityUnits::pct);

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(2);
  }

  return 1;
}

void drive(double inches, double delay) {
  vex::task::sleep(delay);
  resetDriveEncoders = true;
  desiredTurnValue = 666;
  desiredValue = inches / degToInch;
}

void turn(double deg, double delay) {
  vex::task::sleep(delay);
  resetDriveEncoders = true;
  desiredValue = 0;
  desiredTurnValue = deg;
}

void autonomous(void) {
  vex::task runDrivePID(drivePID);
  drive(60, 0);
  // turn(91, 3000);

  // drive(3, 1800);
  // Intake.spinFor(vex::directionType::rev, 1440, degrees, 200, vex::velocityUnits::pct);

  // drive(12, 500);
  // drive(-6, 500);
  // turn(0, 1000);
  // drive(-65, 1800);
  // turn(-90, 3000);
  // drive(18, 1000);
  // turn(-45, 1000);
}


void usercontrol(void) {
  enableDrivePID = false;
  bool openRWing = false;
  bool openLWing = false;
  bool allowA = true;
  bool allowLeft = true;
  bool isBall = false;

  double above1 = 0;
  // Driver Control Main Loop
  while (1) {
    int leftStick = Controller1.Axis3.value();
    int rightStick = Controller1.Axis2.value();
    bool L1 = Controller1.ButtonL1.pressing();
    bool L2 = Controller1.ButtonL2.pressing();
    bool R2 = Controller1.ButtonR2.pressing();
    bool R1 = Controller1.ButtonR1.pressing();
    bool A = Controller1.ButtonA.pressing();
    bool left = Controller1.ButtonLeft.pressing();
    int stickDeadZone = 5;

    if(leftStick > 0) {
      leftStick = (leftStick * leftStick) / 200;
    } else if(leftStick < 0) {
      leftStick = (leftStick * leftStick) / -200;
    }

   if(rightStick > 0) {
      rightStick = (rightStick * rightStick) / 200;
    } else if(leftStick < 0) {
      rightStick = (rightStick * rightStick) / -200;
    }

    // Left wheel control
    if(abs(leftStick) > stickDeadZone) { // Check dead zone
      FL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      ML.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      BL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
    }
    else {
      FL.stop(coast);
      ML.stop(coast);
      BL.stop(coast);
    }

    // Right wheel control
    if(abs(rightStick) > stickDeadZone) {
      FR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      MR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      BR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
    }
    else {
      FR.stop(coast);
      MR.stop(coast);
      BR.stop(coast);
    }

    cout << Intake.current() << endl;
    if(!isBall && !L1) {
      Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(L1) {
      Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      isBall = false;
      matchTime = 0;
    }
    else {
      Intake.stop(coast);
    }

    if(matchTime > 1000) {
      if(Intake.current() >= 1) {
        above1++;
      }
    }
    if(above1 > 3) {
      isBall = true;
      above1 = 0;
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
      Intake.spin(vex::directionType::rev, 0, vex::velocityUnits::pct);
    } else {
      Cata.stop();
    }

    if(R2 && allowA) {
      allowA = false;
      if(openRWing) {
        rightWing.set(true);
        leftWing.set(true);
      }
      else {
        rightWing.set(false);
        leftWing.set(false);
      }
      openRWing = !openRWing;
    }
    else if(!R2) {
      allowA = true;
    }

    matchTime += 20;
    wait(20, msec); // Sleep the task for a short amount of time to
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
