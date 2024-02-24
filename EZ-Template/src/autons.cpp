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
  // --------- Far Side Quals ---------
  // Score alliance triball
  // chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 0);
  // chassis.pid_drive_set(32_in, DRIVE_SPEED);
  // pros::Task::delay(800);
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_drive_set(28_in, DRIVE_SPEED);
  // intake.set_value(true);
  // pros::Task::delay(600);
  // chassis.pid_drive_set(-21_in, DRIVE_SPEED);
  // intake.set_value(false);
  // pros::Task::delay(600);
  // chassis.pid_turn_set(-135_deg, TURN_SPEED);
  // pros::Task::delay(800);

  // // Go to bar triball
  // chassis.pid_swing_set(ez::LEFT_SWING, -90_deg, SWING_SPEED, SWING_SPEED/1.9);
  // pros::Task::delay(800);
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // pros::Task::delay(900);
  // intake.set_value(true);
  // chassis.pid_drive_set(7_in, DRIVE_SPEED/1.2);
  // pros::Task::delay(900);
  // intake.set_value(false);

  // // Score Bar Triball

  // chassis.drive_angle_set(-90_deg);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(-39_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_turn_set(-25_deg, TURN_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(43_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_swing_set(ez::LEFT_SWING, 90, SWING_SPEED, 0);
  // pros::Task::delay(1000);
  // intake.set_value(true);
  // chassis.pid_drive_set(14_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  // pros::Task::delay(1000);

  // // Get barrier triball
  // chassis.pid_turn_set(-124_deg, TURN_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(31_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(3_in, DRIVE_SPEED/2);
  // pros::Task::delay(300);
  // intake.set_value(false);
  // pros::Task::delay(300);
  // chassis.pid_drive_set(-35_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_turn_set(75_deg, TURN_SPEED/1.3);
  // pros::Task::delay(1000);
  // chassis.pid_drive_set(15_in, DRIVE_SPEED);
  // pros::Task::delay(1000);

  // chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, SWING_SPEED * 0.55);

  // --------- Skills ---------
  // chassis.pid_turn_set(-26_deg, TURN_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_drive_set(-5_in, DRIVE_SPEED/2);
  // pros::Task::delay(300);
  // left_cata.move(127);
  // right_cata.move(127);
  // pros::Task::delay(12000);
  // chassis.pid_drive_set(7_in, DRIVE_SPEED);
  // left_cata.move(0);
  // right_cata.move(0);
  // pros::Task::delay(380);
  // chassis.pid_swing_set(ez::LEFT_SWING, 45, SWING_SPEED, -SWING_SPEED * 0.5);
  // pros::Task::delay(420);
  // chassis.pid_swing_set(ez::RIGHT_SWING, 0, SWING_SPEED, SWING_SPEED * 0.18);
  // pros::Task::delay(420); 
  // chassis.pid_drive_set(78_in, DRIVE_SPEED, true);
  // pros::Task::delay(1100);
  // chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, SWING_SPEED * 0.5);
  // pros::Task::delay(600);
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_turn_set(-160, TURN_SPEED);
  // pros::Task::delay(410);
  // chassis.pid_drive_set(40_in, DRIVE_SPEED/1.3);
  // pros::Task::delay(300);
  // wings.set_value(true);
  // pros::Task::delay(420);
  // chassis.pid_turn_set(-90_deg, TURN_SPEED/1.3);
  // pros::Task::delay(450);
  // chassis.pid_drive_set(12_in, DRIVE_SPEED/1.2);
  // pros::Task::delay(350);
  // chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.4);
  // pros::Task::delay(900);
  // chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  // pros::Task::delay(500);
  // chassis.pid_drive_set(20_in, DRIVE_SPEED);
  // pros::Task::delay(500);
  // wings.set_value(false);
  // chassis.pid_drive_set(-40_in, DRIVE_SPEED);
  // pros::Task::delay(600);
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(36_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_turn_set(0_deg, TURN_SPEED/1.3);
  // pros::Task::delay(350);
  // wings.set_value(true);
  // pros::Task::delay(150);
  // chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.1);
  // pros::Task::delay(700);
  // chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.38);
  // pros::Task::delay(800);
  // chassis.pid_drive_set(20_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(22_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // wings.set_value(false);
  // chassis.pid_drive_set(-140_in, DRIVE_SPEED);
  // pros::Task::delay(2150);
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_drive_set(9_in, DRIVE_SPEED);
  // pros::Task::delay(600);
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // pros::Task::delay(450);
  // chassis.pid_drive_set(-54_in, DRIVE_SPEED);
  // pros::Task::delay(1000);
  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(-14_in, DRIVE_SPEED);
  // pros::Task::delay(700);

  // // Movement 2
  // chassis.pid_turn_set(-26_deg, TURN_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_drive_set(-5_in, DRIVE_SPEED/2);
  // left_cata.move(127);
  // right_cata.move(127);
  // pros::Task::delay(12000);
  // chassis.pid_drive_set(7_in, DRIVE_SPEED);
  // left_cata.move(0);
  // right_cata.move(0);
  // pros::Task::delay(380);
  // chassis.pid_swing_set(ez::LEFT_SWING, 45, SWING_SPEED, -SWING_SPEED * 0.5);
  // pros::Task::delay(420);
  // chassis.pid_swing_set(ez::RIGHT_SWING, 0, SWING_SPEED, SWING_SPEED * 0.18);
  // pros::Task::delay(420); 
  // chassis.pid_drive_set(78_in, DRIVE_SPEED, true);
  // pros::Task::delay(1100);
  // chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, SWING_SPEED * 0.5);
  // pros::Task::delay(600);
  // chassis.pid_drive_set(24_in, DRIVE_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_drive_set(-20_in, DRIVE_SPEED);
  // pros::Task::delay(350);
  // chassis.pid_turn_set(-160, TURN_SPEED);
  // pros::Task::delay(410);
  // chassis.pid_drive_set(40_in, DRIVE_SPEED/1.3);
  // pros::Task::delay(300);
  // wings.set_value(true);
  // pros::Task::delay(420);
  // chassis.pid_turn_set(-90_deg, TURN_SPEED/1.3);
  // pros::Task::delay(450);
  // chassis.pid_drive_set(12_in, DRIVE_SPEED/1.2);
  // pros::Task::delay(350);
  // chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.4);
  // pros::Task::delay(900);
  // chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  // pros::Task::delay(500);
  // chassis.pid_drive_set(20_in, DRIVE_SPEED);
  // pros::Task::delay(500);
  // wings.set_value(false);
  // chassis.pid_drive_set(-40_in, DRIVE_SPEED);
  // pros::Task::delay(600);
  // chassis.pid_turn_set(-90_deg, TURN_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(36_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_turn_set(0_deg, TURN_SPEED/1.3);
  // pros::Task::delay(350);
  // wings.set_value(true);
  // pros::Task::delay(150);
  // chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.1);
  // pros::Task::delay(700);
  // chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED/1.6, (SWING_SPEED/1.6) * 0.38);
  // pros::Task::delay(800);
  // chassis.pid_drive_set(20_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(22_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  // pros::Task::delay(700);
  // wings.set_value(false);
}
