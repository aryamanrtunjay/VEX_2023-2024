#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FL = motor(PORT1, ratio6_1, true);
motor FR = motor(PORT20, ratio6_1, false);
motor ML = motor(PORT9, ratio6_1, true);
motor MR = motor(PORT17, ratio6_1, false);
motor BL = motor(PORT15, ratio6_1, true);
motor BR = motor(PORT5, ratio6_1, false);
controller Controller1 = controller(primary);
motor Arm = motor(PORT8, ratio36_1, false);
motor Flytake = motor(PORT18, ratio6_1, false);
digital_out Wing = digital_out(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT12);
digital_out PTO = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}