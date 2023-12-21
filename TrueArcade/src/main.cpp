#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
pros::Motor left_back(1, pros::E_MOTOR_GEARSET_06, false);
pros::Motor left_middle(2, pros::E_MOTOR_GEARSET_06, false);
pros::Motor left_front(3, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_back(4, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_middle(5, pros::E_MOTOR_GEARSET_06, false);
pros::Motor right_front(6, pros::E_MOTOR_GEARSET_06, false);

pros::MotorGroup left_side_motors({left_back, left_middle, left_front});
pros::MotorGroup right_side_motors({right_back, right_middle, right_front});

lemlib::Drivetrain_t drivetrain {
	&left_side_motors,
	&right_side_motors,
	25,
	3.25,
	360
};

pros::Imu inertial_sensor(7);

lemlib::OdomSensors_t sensors {
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&inertial_sensor
};

// forward/backward PID
lemlib::ChassisController_t lateralController {
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void screen() {
	while(true) {
		lemlib::Pose pose = chassis.getPose();
		pros::lcd::print(0, "x: %f", pose.x);
		pros::lcd::print(1, "y: %f", pose.y);
		pros::lcd::print(2, "heading: %f", pose.theta);
		pros::delay(10);
	}
}


void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	chassis.setPose(0,0,0);
	pros::Task screenTask(screen);
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
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
		int horizontal = master.get_analog(ANALOG_LEFT_X);
		int vertical = master.get_analog(ANALOG_LEFT_Y);
		lemlib::Pose pose = chassis.getPose();
		chassis.moveTo(pose.x + horizontal, pose.y - vertical, 30);
		pros::delay(20);
	}
}
