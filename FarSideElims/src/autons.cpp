#include "main.h"
#define WINGS_PORT 'A'
#define INTAKE_PORT 'H'
#define OPTICAL_PORT 5
#define ROTATION_PORT 20

/////
// For installation, upgrading, documentations and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;  
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(0.8, 0.01, 0);
  chassis.pid_drive_constants_set(700, 50, 0);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);
}

void AllAutons() {
  pros::Motor left_cata(4, true);
  pros::Motor right_cata(10, false);

  chassis.pid_heading_constants_set(3, 0, 20);
  chassis.pid_drive_constants_set(10, 0, 100);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);

  // chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 45);
  // chassis.pid_wait();

  pros::ADIDigitalOut wings (WINGS_PORT);
  pros::ADIDigitalOut intake (INTAKE_PORT);

  intake.set_value(true);
  chassis.drive_angle_set(-98_deg);
  // Drive to descore
  chassis.pid_drive_set(-32.4_in, DRIVE_SPEED, false);
  pros::Task::delay(500);
  wings.set_value(true);
  pros::Task::delay(300);
  chassis.pid_turn_set(-120_deg, TURN_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(-22.5_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(-180_deg, TURN_SPEED, false);
  wings.set_value(false);
  pros::Task::delay(800);   
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(15_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(0_deg, TURN_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(15_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(-92_deg, TURN_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  pros::Task::delay(2000);
  

}
