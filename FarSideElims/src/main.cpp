#include "main.h"


#define WINGS_PORT 'A'
#define INTAKE_PORT 'H'
#define HANG_PORT 'C'
#define OPTICAL_PORT 5
#define ROTATION_PORT 20

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
pros::Motor left_cata(4, true);
pros::Motor right_cata(10, false);

// Chassis constructor
ez::Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  {1, 2, -3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is used as the sensor
  ,{-9, 8, -7}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
  ,2.75

  // Cartridge RPM
  ,600

  // External Gear Ratio (MUST BE DECIMAL) This is WHEEL GEAR / MOTOR GEAR
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 84/36 which is 2.333
  // eg. if your drive is 60:36 where the 36t is powered, your RATIO would be 60/36 which is 0.6
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 36/60 which is 0.6
  ,1
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.opcontrol_curve_default_set(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    Auton("All Autons", AllAutons)
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset(); // Resets PID targets to 0
  chassis.drive_imu_reset(); // Reset gyro position to 0
  chassis.drive_sensor_reset(); // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call(); // Calls selected auton from autonomous selector
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  bool allowWings = false;
  bool wingState = false;
  bool allowHang = false;
  bool cataCocked = false;
  bool first = true;
  int targetCata = 55;

  double rot = 0;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  pros::Controller master (pros::E_CONTROLLER_MASTER);
  pros::ADIDigitalOut hang (HANG_PORT);
  pros::ADIDigitalOut intake (INTAKE_PORT);
  pros::ADIDigitalOut wings (WINGS_PORT);
  pros::Distance dist(OPTICAL_PORT);
  pros::Rotation rotation_sensor(ROTATION_PORT);
  rotation_sensor.reset();
  while (true) {
    int cata = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    int R1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    int R2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    int L1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    int RIGHT = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    int Y = master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    cata = 1;
    chassis.opcontrol_tank(); // Tank control

    if(R2) {
      left_cata.move(83);
      right_cata.move(83);
      pros::Task::delay(1);
    } else {
      left_cata.move(0);
      right_cata.move(0);
    }

    if(allowHang && RIGHT == 1 && Y == 1) {
      allowHang = false;
      hang.set_value(true);
    } else if(RIGHT == 0 || Y == 0) {
      allowHang = true;
    }

    if(allowWings && R1 == 1) {
      allowWings = false;
      wings.set_value(!wingState);
      wingState = !wingState;
    } else if(R1 == 0) {
      allowWings = true;
    }
    if(L1 == 1) {
      intake.set_value(false);
    } else {
      intake.set_value(true);
    }
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
