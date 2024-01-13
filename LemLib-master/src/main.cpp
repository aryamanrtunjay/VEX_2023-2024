#include "main.h"
#include "lemlib/api.hpp"
#define DIGITAL_SENSOR_PORT 'A'

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors

pros::Motor lF(7, pros::E_MOTOR_GEARSET_06);
pros::Motor lM(8, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-9, pros::E_MOTOR_GEARSET_06);
pros::Motor rF(-4, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(-3, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(2, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor cataLeft(20, pros::E_MOTOR_GEARSET_36);
pros::Motor cataRight(-11, pros::E_MOTOR_GEARSET_36);


// motor groups

pros::Motor lT(10, pros::E_MOTOR_GEARSET_06);
pros::Motor rT(-1, pros::E_MOTOR_GEARSET_06);// left back motor. port 1, reversed
pros::MotorGroup leftMotors({lT, lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rT, rF, rM, rB}); // right motor group

/*
// motor groups
pros::MotorGroup leftMotors({lF, lM, lB});
pros::MotorGroup rightMotors({rF, rM, rB});
*/
// Inertial Sensor on port 2
pros::Imu imu(6);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
// pros::Rotation horizontalEnc(15, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 3.25" omnis
                              600, // drivetrain rpm is 360
                              8 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(15, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             1, // anti windup
                                             0.1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             10 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);


double PI = 3.14159265;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT);
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(0, 10, 90, 4000);
    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    double currX = 0;
    double currY = 0;
    double theta = 0;
    lemlib::Pose pose = chassis.getPose();
    chassis.moveToPose(currX - 20, currY + 36, theta - 45, 1000);
    currX -= 20;
    currY += 36;
    theta -= 45;
    pose = chassis.getPose();
    chassis.moveToPose(currX - 10, currY + 20, theta + 45, 2000);
    // chassis.moveToPose(pose.x, pose.y + 36, pose.theta + 45, 2000);
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 * */
// Tank Code
// void opcontrol() {
//     while(true) {
//         double leftJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         double rightJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
//         double left = pow(1.03888, abs(leftJoy));
//         double right = pow(1.03888, abs(rightJoy));
//         rightMotors.move(right * rightJoy / abs(rightJoy));
//         leftMotors.move(left * leftJoy / abs(leftJoy));
//         pros::delay(2);
//     }
// }

// Arcade
// void opcontrol() {
//     while(true) {
//         double vert = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         double hor = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
//         double turn = pow(hor / 127, 2) *127 * abs(hor) / hor;
//         leftMotors.move(vert + hor);
//         rightMotors.move(vert - hor);
//         pros::delay(2);
//     }
// }

// Other Arcade
// void opcontrol() {
//     while(true) {
//         double vert = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         double hor = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

//     }
// }
// Field Oriented
void opcontrol() {
    pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT);
    while (true) {
        double vertical = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double horizontal = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        bool backwards = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool A = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool activateWings = true;
        bool allowWings = true;
        double target_heading = atan2(vertical, horizontal);
        if(backwards) {
            target_heading += PI;
        }
        double magnitude = sqrt(vertical * vertical + horizontal * horizontal);
        lemlib::Pose pose = chassis.getPose(true);
        double currentHeading = pose.theta;
        double deltaHeading = target_heading - currentHeading;
        do {
            if(abs(deltaHeading) > PI) {
                deltaHeading += -2 * PI * abs(deltaHeading) / deltaHeading;
            } 
        } while(abs(deltaHeading) > PI);
        double leftMotorPower = backwards ? -127: 127;
        double rightMotorPower = backwards ? -127: 127;
        if(deltaHeading > 0) {
            rightMotorPower -= (backwards ? -1: 1) * deltaHeading * 508 / PI;
        } else if(deltaHeading < 0)  {
            leftMotorPower += (backwards ? -1: 1) * deltaHeading * 508 / PI;
        }
        if(abs(rightMotorPower) > 127) {
            rightMotorPower = 127 * abs(rightMotorPower) / rightMotorPower;
        }
        if(abs(leftMotorPower) > 127) {
            leftMotorPower = 127 * abs(leftMotorPower) / leftMotorPower;
        }
        leftMotors.move(leftMotorPower * magnitude / 127);
        rightMotors.move(rightMotorPower * magnitude / 127);

        if(backwards && allowWings) {
            allowWings = false;
            // pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT);
            wings.set_value(activateWings);
            activateWings = !activateWings;
        }
        else {
            allowWings = true;
        }

        pros::delay(2);
    }
}
