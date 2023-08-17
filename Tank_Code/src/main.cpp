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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}


void usercontrol(void) {
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
