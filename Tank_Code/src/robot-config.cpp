#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FL = motor(PORT18, ratio6_1, true);
motor FR = motor(PORT6, ratio6_1, false);
motor ML = motor(PORT20, ratio6_1, true);
motor MR = motor(PORT10, ratio6_1, false);
motor BL = motor(PORT16, ratio6_1, true);
motor BR = motor(PORT19, ratio6_1, false);
controller Controller1 = controller(primary);
motor Cata = motor(PORT14, ratio36_1, false);
motor Intake = motor(PORT1, ratio6_1, false);

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