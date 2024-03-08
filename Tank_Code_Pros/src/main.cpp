#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor LF(2);
pros::Motor RF(10);
pros::Motor LM(20);
pros::Motor RM(19);
pros::Motor LB(5);
pros::Motor RB(1);
pros::Motor cata(14);
pros::Motor intake(3);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	LM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	RM.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	LB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	cata.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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
 * task, not resume it from where it left off.`	
 */

 
void opcontrol() {
	while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		int cataU = master.get_digital(DIGITAL_R1);
		int cataD = master.get_digital(DIGITAL_R2);
		int intakeU = master.get_digital(DIGITAL_L1);
		int intakeD = master.get_digital(DIGITAL_L2);

		int intakeVolt = 0;
		if(intakeU >= 0) {
			intakeVolt = 12000;
		}

		LF.move(50);
		RF.move(50);

		intake.move_voltage(intakeVolt);

		pros::delay(10);
	}
}
