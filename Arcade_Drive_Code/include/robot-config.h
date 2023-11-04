using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FL;
extern motor FR;
extern motor ML;
extern motor MR;
extern motor BL;
extern motor BR;
extern motor Cata;
extern motor Intake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );