#include "main.h"
#include "lemlib/api.hpp"
#define WINGS_PORT 'A'
#define HANG_PORT 'B'
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors

pros::Motor lF(1, pros::E_MOTOR_GEARSET_06);
pros::Motor lM(2, pros::E_MOTOR_GEARSET_06); // left middle motor. port 11, reversed
pros::Motor lB(-3, pros::E_MOTOR_GEARSET_06);
pros::Motor lT(4, pros::E_MOTOR_GEARSET_06);
pros::Motor rF(-10, pros::E_MOTOR_GEARSET_06); // right front motor. port 2
pros::Motor rM(-9, pros::E_MOTOR_GEARSET_06); // right middle motor. port 11
pros::Motor rB(8, pros::E_MOTOR_GEARSET_06); // right back motor. port 13
pros::Motor rT(-7, pros::E_MOTOR_GEARSET_06);// left back motor. port 1, reversed
pros::Motor cataLeft(-20, pros::E_MOTOR_GEARSET_36);
pros::Motor cataRight(11, pros::E_MOTOR_GEARSET_36);


// motor groups

pros::MotorGroup leftMotors({lF, lT, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rT, rM, rB}); // right motor group

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
lemlib::ControllerSettings linearController(600, // proportional gain (kP)
                                            50, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            200 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0.9, // proportional gain (kP)
                                             0.01, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             1, // anti windup
                                             0.1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             100 // maximum acceleration (slew)
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
double cX = 0;
double cY = 0;
double cT = 0;
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    pros::ADIDigitalOut wings (WINGS_PORT);
    pros::ADIDigitalOut hang (HANG_PORT);
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

void moveBot(double x, double y, double theta, int timeout, bool fwd = true) {
    chassis.moveToPose(cX + x, cY + y, cT + theta, timeout, {.forwards = !fwd});
    cX += x;
    cY += y;
    cT += theta;
}

void moveBot(double x, double y, int timeout, bool fwd = true) {
    chassis.moveToPoint(cX + x, cY + y, timeout);
    cX += x;
    cY += y;

}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    pros::ADIDigitalOut wings (WINGS_PORT);
    moveBot(-19.67, 48, 0.0, 1800, false);
    moveBot(-2, -35.59, 353.33, 1000, true);
    moveBot(0, 0, 80, 1500, false);

    // -- Start Push 1

    //moveBot(0, 0, 110, 1000, false);
    moveBot(20.48, -5.69, 12.4, 2000, false);
    moveBot(84, 0, 0, 1500, false);
    moveBot(0, 0, 135, 1000, false);
    moveBot(16, 45, -45, 1100, true);
    moveBot(0, -29, 0, 1100, false);
    moveBot(0, 0, -70, 1000, false);
    moveBot(-48, 18, 25, 1500, true);

    wings.set_value(true);
    moveBot(0, 0, 90, 1000, false);
    moveBot(16, 16, 45, 1500, true);
    moveBot(35, 0, 60, 1500, true);
    moveBot(-24, 0, 0, 1500, false);

    // // Return to start

    // moveBot(-11, 0, 0, 1500, false);
    // wings.set_value(false);
    // moveBot(0, 0, 90, 1000, false);
    // moveBot(0, -60, 0, 2000, true);
    // moveBot(0, 0, 90, 1000, true);
    // moveBot(0, -70, 0, 2000, true);
    // moveBot(0, 0, 90, 1000, false);
    // moveBot(-23, 10, -120, true);




    // moveBot(23.31, 31.47, -80.0, 3000, false);
    // moveBot(-14.18, -19.1, 33.33, 3000, true);
    // moveBot(-35.75, 11.33, 56.67, 3000, true);
    // moveBot(2.33, 13.99, 116.67, 3000, true);
    // moveBot(8.16, 7.81, 36.66, 3000, true);
    // moveBot(25.07, 2.1, 26.67, 3000, true);
    // moveBot(-37.89, -0.71, -3.33, 3000, false);
    // moveBot(5.63, 14.76, -53.34, 3000, true);
    // moveBot(8.75, 5.83, 36.67, 3000, true);
    // moveBot(24.28, 0.19, 20.0, 3000, true);
    // moveBot(-12.6, 0.0, -180.0, 3000, false);
    // moveBot(-0.22, -68.38, -90.0, 3000, true);
    // moveBot(-89.37, 4.4, 93.33, 3000, true);
    // moveBot(92.48, -3.69, -13.33, 3000, false);
    // moveBot(23.31, 31.47, -80.0, 3000, false);
    // moveBot(-14.18, -19.1, 33.33, 3000, true);
    // moveBot(-35.75, 11.33, 56.67, 3000, true);
    // moveBot(2.33, 13.99, 116.67, 3000, true);
    // moveBot(8.16, 7.81, 36.66, 3000, true);
    // moveBot(25.07, 2.1, 26.67, 3000, true);
    // moveBot(-37.89, -0.71, -3.33, 3000, false);
    // moveBot(5.63, 14.76, -53.34, 3000, true);
    // moveBot(8.75, 5.83, 36.67, 3000, true);
    // moveBot(24.28, 0.19, 20.0, 3000, true);
    // moveBot(-12.6, 0.0, -180.0, 3000, false);
    // moveBot(-0.22, -68.38, -90.0, 3000, true);

    // moveBot(0, -32, 0, 700, true);
    // moveBot(-36, -20, 90, 2500, true);
    // wings.set_value(true);
    // moveBot(12, 0, 90, 1500, false);
    // wings.set_value(false);
    // moveBot(-12, 11, 180, 3000, true);
    // moveBot(0, 0, 215, 2000, true);
    // moveBot(5, 2, 215, 1000, true);
    // wings.set_value(true);
}


void opcontrol() {
    bool wingState = false;
    pros::ADIDigitalOut hang (HANG_PORT);
    pros::ADIDigitalOut wings (WINGS_PORT);
    while(true) {
        double leftJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double rightJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        double w = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        double h = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        // double left = pow(1.03888, abs(leftJoy)) * abs(leftJoy) / leftJoy;
        // double right = pow(1.0388, abs(rightJoy)) * abs(rightJoy) / rightJoy;
        double left = pow(leftJoy / 127, 3) * 127;
        double right = pow(rightJoy / 127, 3) * 127;
        leftMotors.move(left);
        rightMotors.move(right);

        if(w) {
            if(wingState) {
                wings.set_value(false);
                wingState = false;
            } else {
                wings.set_value(true);
                wingState = true;
            }
        } 

        if(h) {
            hang.set_value(false);
        }

        pros::delay(2);
    };
}

/**
 * Runs in driver control
 * */
// Tank Code
// void opcontrol() {
//     bool allowWings = true;
//     bool activateWings = false;
//     pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT);
//     while(true) {
//         double leftJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         double rightJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
//         double left = pow(1.03888, abs(leftJoy));
//         double right = pow(1.03888, abs(rightJoy));

//         bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
//         bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

//         rightMotors.move(right * rightJoy / abs(rightJoy));
//         leftMotors.move(left * leftJoy / abs(leftJoy));

//         if(L1 && allowWings) {
//             allowWings = false;
//             wings.set_value(!activateWings);
//             activateWings = !activateWings;
//             cataLeft.move(127);
//             cataRight.move(127);
//         }
//         else if(!L1) {
//             allowWings = true;
//         }

//         if(R1) {
//             cataLeft.move(127);
//             cataRight.move(127);
//         }

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
// void opcontrol() {
//     pros::ADIDigitalOut wings (DIGITAL_SENSOR_PORT);
//     while (true) {
//         double vertical = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         double horizontal = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
//         bool backwards = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
//         bool A = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
//         bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
//         bool activateWings = true;
//         bool allowWings = true;
//         double target_heading = atan2(vertical, horizontal);
//         if(backwards) {
//             target_heading += PI;
//         }
//         double magnitude = sqrt(vertical * vertical + horizontal * horizontal);
//         lemlib::Pose pose = chassis.getPose(true);
//         double currentHeading = pose.theta;
//         double deltaHeading = target_heading - currentHeading;
//         do {
//             if(abs(deltaHeading) > PI) {
//                 deltaHeading += -2 * PI * abs(deltaHeading) / deltaHeading;
//             } 
//         } while(abs(deltaHeading) > PI);
//         double leftMotorPower = backwards ? -127: 127;
//         double rightMotorPower = backwards ? -127: 127;
//         if(deltaHeading > 0) {
//             rightMotorPower -= (backwards ? -1: 1) * deltaHeading * 508 / PI;
//         } else if(deltaHeading < 0)  {
//             leftMotorPower += (backwards ? -1: 1) * deltaHeading * 508 / PI;
//         }
//         if(abs(rightMotorPower) > 127) {
//             rightMotorPower = 127 * abs(rightMotorPower) / rightMotorPower;
//         }
//         if(abs(leftMotorPower) > 127) {
//             leftMotorPower = 127 * abs(leftMotorPower) / leftMotorPower;
//         }
//         leftMotors.move(leftMotorPower * magnitude / 127);
//         rightMotors.move(rightMotorPower * magnitude / 127);

//         pros::delay(2);
//     }
// }
