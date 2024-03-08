#include "main.h"
#define WINGS_PORT 'A'
#define INTAKE_PORT 'H'

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
  
  // Disrupt balls
  chassis.pid_turn_set(22, TURN_SPEED, false);
  chassis.pid_drive_set(36, DRIVE_SPEED, false);
  pros::Task::delay(850);
  wings.set_value(true);
  chassis.pid_swing_set(ez::LEFT_SWING, 90, SWING_SPEED, SWING_SPEED * 0.2);
  pros::Task::delay(700);
  chassis.pid_drive_set(26, DRIVE_SPEED, false);
  pros::Task::delay(700);
  wings.set_value(false);

  // Return
  pros::Task::delay(300);
  chassis.pid_turn_set(45, TURN_SPEED, false);
  chassis.pid_drive_set(-46, DRIVE_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_turn_set(20, TURN_SPEED, false);
  chassis.pid_drive_set(-22, DRIVE_SPEED, false);
  pros::Task::delay(400);
  wings.set_value(true);
  pros::Task::delay(700);
  chassis.pid_turn_set(-45, TURN_SPEED, false);
  chassis.pid_drive_set(-20, DRIVE_SPEED, false);

  pros::Task::delay(1000);
  wings.set_value(false);
  chassis.pid_turn_set(-130, TURN_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_turn_set(-90, TURN_SPEED, false);
  pros::Task::delay(1000);
  chassis.pid_drive_set(-39, DRIVE_SPEED, false);
}
