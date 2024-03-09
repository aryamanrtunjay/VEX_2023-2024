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
  chassis.pid_heading_constants_set(3, 0, 20);
  chassis.pid_drive_constants_set(10, 0, 100);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);
  
}

void AllAutons() {
  chassis.pid_heading_constants_set(3, 0, 20);
  chassis.pid_drive_constants_set(10, 0, 100);
  chassis.pid_turn_constants_set(3, 0, 20);
  chassis.pid_swing_constants_set(5, 0, 30);

  chassis.pid_turn_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_swing_exit_condition_set(300_ms, 3_deg, 500_ms, 7_deg, 750_ms, 750_ms);
  chassis.pid_drive_exit_condition_set(300_ms, 1_in, 500_ms, 3_in, 750_ms, 750_ms);

  chassis.slew_drive_constants_set(7_in, 80);

  pros::ADIDigitalOut wings (WINGS_PORT);
  pros::ADIDigitalOut intake (INTAKE_PORT);
  chassis.drive_angle_set(-98_deg);

  chassis.pid_drive_set(2_in, DRIVE_SPEED, false);
  pros::Task::delay(500);
  intake.set_value(true);
  pros::Task::delay(500);
  // Drive to descore
  chassis.pid_drive_set(-36_in, DRIVE_SPEED, false);
  pros::Task::delay(1500);
  wings.set_value(true);
  pros::Task::delay(300);
  chassis.pid_turn_set(-120_deg, TURN_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_drive_set(-23_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(-175_deg, TURN_SPEED, false);
  wings.set_value(false);
  pros::Task::delay(800);   
  chassis.pid_drive_set(-18_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(-180_deg, TURN_SPEED, false);
  pros::Task::delay(200);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_turn_set(0_deg, TURN_SPEED, false);
  pros::Task::delay(1200);
  intake.set_value(false);
  chassis.pid_drive_set(20_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, false);
  pros::Task::delay(800);
  intake.set_value(true);
  chassis.pid_turn_set(-65_deg, TURN_SPEED, false);
  pros::Task::delay(800);
  chassis.pid_drive_set(54_in, DRIVE_SPEED, false);
  pros::Task::delay(2000);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED, false);
  pros::Task::delay(500);
  chassis.pid_turn_set(0_deg, TURN_SPEED, false);
  pros::Task::delay(500);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  pros::Task::delay(800);
  wings.set_value(true);
  chassis.pid_drive_set(32_in, DRIVE_SPEED, false);
  pros::Task::delay(1200);
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, false);
  wings.set_value(false);
  pros::Task::delay(700);
  chassis.pid_turn_set(-70_deg, TURN_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, false);
  pros::Task::delay(1500);
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  pros::Task::delay(500);
  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_drive_set(34_in, DRIVE_SPEED, false);
  pros::Task::delay(1500);
  chassis.pid_drive_set(-2_in, DRIVE_SPEED, false);
  pros::Task::delay(500);

}
