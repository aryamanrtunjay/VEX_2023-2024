using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FL;
extern motor FR;
extern motor ML;
extern motor MR;
extern motor BL;
extern motor BR;
extern controller Controller1;
extern motor Arm;
extern motor Flytake;
extern digital_out rightWing;
extern digital_out leftWing;
extern inertial Inertial;
extern digital_out PTO;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );