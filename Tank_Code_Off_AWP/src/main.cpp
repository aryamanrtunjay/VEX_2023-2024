// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   C               
// Inertial             inertial      12              
// PTO                  digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   B               
// Inertial             inertial      12              
// PTO                  digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   G               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         19              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         13              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   B               
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
  FL.setPosition(0, degrees);
  ML.setPosition(0, degrees);
  BL.setPosition(0, degrees);
  FR.setPosition(0, degrees);
  MR.setPosition(0, degrees);
  BR.setPosition(0, degrees);
  Flytake.setPosition(0, degrees);
  Arm.setPosition(0, degrees);

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
bool enableFlywheelPID = true;

// Settings
// double kU = 0.30;
// double pU = 0.70;
// double kP = 0.6 * kU;
// double kI = 0.3 * kU / pU;
// double kD = 0.075 * kU / pU;

double kP = 0.068;
double kI = 0.0;
double kD = 0;

double turnkP = 6;
double turnkI = 0.00001;
double turnkD = 0.4;

double flykP = 1;
double flykI = 0.0;
double flykD = 0.0;
// Autonomous Settings
double desiredValue = 0;
int desiredTurnValue = 0;
double desiredRPM = 0;

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

    int leftMotorPositionDeg = (ML.position(degrees) + BL.position(degrees) + FL.position(degrees));
    int rightMotorPositionDeg = (MR.position(degrees) + BR.position(degrees) + FR.position(degrees));

    int averagePosition = (leftMotorPositionDeg + rightMotorPositionDeg) / 6;
    // int averagePosition = averagePositionDeg / degToInch;

    error = desiredValue - averagePosition;
    derivative = error - prevError;
    totalError += error;

    double latMotorPower = ((error * kP) + (totalError * kI) + abs((derivative * kD))); 
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
    // -------------------------- Turning Movement PID -------------------------- //
    double speedLimit = 0.5;
    FL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    BL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    MR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    BR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    FR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(2);
  }

  return 1;
}

int flywheelPID() {
  double error = 0;
  double prevError = 0;
  double totalError = 0;
  double motorPower = 0;
  while(enableFlywheelPID) {
    double vel = Flytake.velocity(vex::velocityUnits::rpm);
    error = desiredRPM - vel;
    derivative = error - prevError;
    totalError += error;

    motorPower += ((error * flykP) + (totalError * flykI) + (derivative * flykD));
    cout << error << endl;
    
    Flytake.spin(vex::directionType::fwd, motorPower, vex::velocityUnits::pct);
    wait(20, msec);
    prevError = error;
  }
  
  return 1;
}

void drive(double inches, double delay) {
  vex::task::sleep(delay);
  resetDriveEncoders = true;
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
  resetDriveEncoders = true;
  desiredValue = 0;
  Arm.spinFor(vex::directionType::fwd, 270, vex::rotationUnits::deg, 100, vex::velocityUnits::pct, true);
  wait(20, msec);
  Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  drive(8.24, 0);
  drive(-43, 800);
  turn(135, 700);
  drive(22, 600);
  turn(90, 600);
  drive(12, 500);
  Wing.set(true);
  Arm.spinFor(vex::directionType::fwd, 210, vex::rotationUnits::deg, true);
  drive(9, 1000);
  Flytake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  drive(-8, 700);
  Wing.set(false);
  turn(-90, 700);
  Arm.spinFor(vex::directionType::rev, 210, vex::rotationUnits::deg, true);
  Wing.set(false);
  drive(-25, 700);
  drive(5, 500);
  turn(-70, 700);
  drive(17, 600);
  turn(54, 700);
  Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  drive(72, 600);

  turn(1, 1000);
  drive(-40, 600);
  drive(10, 1000);

  turn(-170, 800);
  drive(4, 500);
  Flytake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  drive(10, 500);
  drive(-10, 500);

  turn(-35, 500);
  Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  drive(62, 700);
  turn(145, 1000);
  drive(55, 1000);
  Flytake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  drive(10, 1000);
  drive(-10, 1000);
}

void usercontrol(void) {
  // vex::task runFlywheelPID(flywheelPID);
  vex::task runDrivePID(drivePID);

  enableDrivePID = false;
  enableFlywheelPID = true;
  desiredRPM = 0;
  bool openRWing = false;
  bool openLWing = false;
  bool allowA = true;
  bool allowLeft = true;
  bool isBall = false;
  bool armUp = false;
  bool moveArmUp = false;
  bool moveArmDown = true;

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
    bool right = Controller1.ButtonRight.pressing();
    bool up = Controller1.ButtonUp.pressing();
    bool down = Controller1.ButtonDown.pressing();
    int stickDeadZone = 5;
    bool outtake = false;

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

    if(up) {
      enableDrivePID = true;
      desiredValue = 90;
    }
    else if(right) {
      enableDrivePID = true;
      desiredValue = 180;
    }
    else if(down) {
      enableDrivePID = true;
      desiredValue = -90;
    }
    else if(left) {
      enableDrivePID = true;
      desiredValue = 0;
    }
    else {
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
    }

    if(L1 && !armUp) {
      moveArmUp = true;
    }
    if(L2) {
      if(armUp) {
        moveArmDown = true;
      }
      else { 
        Flytake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      }
    }
    
    if(L1 && moveArmDown) {
      moveArmDown = false;
      moveArmUp = true;
    }
    if(L2 && moveArmUp) {
      moveArmDown = true;
      moveArmUp = false;
    }

    if(!L2 && !armUp) {
      Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }

    if(moveArmUp && Inertial.pitch() < 70) {
      Arm.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      PTO.set(true);
      Flytake.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
    }
    else if(moveArmUp) {
      moveArmUp = false;
      Arm.stop(coast);
      armUp = true;
    }
    if(moveArmDown && Inertial.pitch() > 28) {
      PTO.set(false);
      Arm.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      Flytake.spin(vex::directionType
      ::fwd, 100, vex::velocityUnits::pct);
    }
    else if(moveArmDown) {
      moveArmDown = false;
      Arm.stop(coast);
      armUp = false;
    }
 
    if(R2 && allowA) {
      allowA = false;
      if(openRWing) {
        Wing.set(true);
      }
      else {
        Wing.set(false);
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

// void usercontrol(void) {
//   vex::task runFlywheelPID(flywheelPID);

//   PTO.set(false);
//   desiredRPM = 300;
// }

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  // Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
