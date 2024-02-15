#include "main.h"
#include "lemlib/api.hpp"
#define WINGS_PORT 'A'
#define INTAKE_PORT 'H'
#define HANG_PORT 'C'
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
lemlib::ControllerSettings angularController(0.8, // proportional gain (kP)
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


void moveBot(double x, double y, double theta, int timeout, bool bwd = false) {
    chassis.moveToPose(cX + x, cY + y, cT + theta, timeout, {.forwards = !bwd});
    cX += x;
    cY += y;
    cT += theta;
}

void moveBot(double x, double y, int timeout, bool fwd = true) {
    chassis.moveToPoint(cX + x, cY + y, timeout, !fwd, 21);
    cX += x;
    cY += y;

}

/**
 * Runs during autooveBot(0, 0, -45, 500, true);
    moveBot(18, -18, 0.0, 1000, true);
    moveBot(0, 0, 45, 500, true);oveBot(0, 0, -45, 500, true);
    moveBot(18, -18, 0.0, 1000, true);
    moveBot(0, 0, 45, 500, true);
    moveBot(0, -5, 0, 1000, true);
    moveBot(0, 8, 0, 1000, false);
    moveBot(0, 0, 180, 1000, false);
    moveBot(-24, 25, -90, 4000, true);
    moveBot(-21, 0, 0, 2000, true);
    moveBot(0, 0, 80, 500, false);
    moveBot(0, -5, 0, 1000, true);
    moveBot(0, 8, 0, 1000, false);
    moveBot(0, 0, 180, 1000, false);
    moveBot(-24, 25, -90, 4000, true);
    moveBot(-21, 0, 0, 2000, true);
    moveBot(0, 0, 80, 500, false);
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void CloseSideElims() {
    pros::ADIDigitalOut wings (WINGS_PORT);
    pros::ADIDigitalOut intake (INTAKE_PORT);
    wings.set_value(true);
    moveBot(0, 0, 26, 500, false);
    wings.set_value(false);
    moveBot(21.667, 43.641, 0.0, 1500, false);
    moveBot(19.524, 4.898, 64, 1500, false);
    moveBot(-20.872, -29.939, -90, 2000, true);
    moveBot(-32.135, -4.414, 90, 1500, true);
    moveBot(0, 0, 90, 500, false);
    moveBot(0, 12.5, 0, 750, true);
    moveBot(0, -7, 0, 1000, false);
    
}

void closeSideQuals(pros::ADIDigitalOut wings, pros::ADIDigitalOut intake) {
    moveBot(0, 0, -45, 500, true);
    moveBot(18, -18, 0.0, 1000, true);
    moveBot(0, 0, 45, 500, true);
    moveBot(0, -5, 0, 1000, true);
    moveBot(0, 8, 0, 1000, false);
    moveBot(0, 0, 180, 1000, false);
    moveBot(-24, 25, -90, 4000, true);
    moveBot(-21, 0, 0, 2000, true);
    moveBot(0, 0, 80, 500, false);
}

void FarSideQuals(pros::ADIDigitalOut wings, pros::ADIDigitalOut intake) {
    moveBot(0, 2.5, 0, 600, false);
    moveBot(0, -29, 3000, true);
    moveBot(26, -27, -45, 1500, true);
    moveBot(0, 0, -45, 1400, false);
    moveBot(-10, 0, 0, 1400, false);
    moveBot(30, 0, 0, 1400, true);
    moveBot(-25, 0, 0, 1400, false);
    moveBot(0, 0, 180, 1500, false);
    moveBot(-5, 0, 0, 1500, true);
    moveBot(20, 0, 0, 1500, false);
    moveBot(-25, 0, 0, 1500, true);
    moveBot(0, 0, -78, 1500, false);
    moveBot(18, 53, 0, 1500, false);
    // moveBot(0, 0, -60, 1000, false);


}

void autonomous() {
    pros::ADIDigitalOut wings (WINGS_PORT);
    pros::ADIDigitalOut intake (INTAKE_PORT);
    wings.set_value(true);
    moveBot(0, 0, 26, 500, false);
    // wings.set_value(false);
    moveBot(21.667, 43.641, 0.0, 1500, false);
    moveBot(19.524, 4.898, 64, 1500, false);
    moveBot(-20.872, -25.939, -90, 2000, true);
    // wings.set_value(false);
    moveBot(-32.135, -4.414, 90, 1500, true);
    moveBot(0, 0, 90, 500, false);
    moveBot(0, 12.5, 0, 750, true);
    moveBot(0, -7, 0, 1000, false);
    
    // moveBot(0, 0, -45, 500, false);
    // moveBot(18, -18, 0.0, 1000, true);
    // moveBot(0, 0, 45, 500, false);
    // moveBot(0, -15, 0.0, 1000, true);
    // moveBot(0, 5, 0.0, 500, false);
    // moveBot(0, 0, -90, 500, false);
    // moveBot(-30, 0, 0.0, 1000, false);
    // moveBot(-12, -24, 0.0, 2000, false);
}


void opcontrol() {
    bool wingState = false;
    bool allowWings = false;
    bool allowHang = false;
    pros::ADIDigitalOut hang (HANG_PORT);
    pros::ADIDigitalOut intake (INTAKE_PORT);
    pros::ADIDigitalOut wings (WINGS_PORT);
    while(true) {
        double leftJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double rightJoy = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int w = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        int h = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        int y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        int r = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        // double left = pow(1.03888, abs(leftJoy)) * abs(leftJoy) / leftJoy;
        // double right = pow(1.0388, abs(rightJoy)) * abs(rightJoy) / rightJoy;
        double left = pow(leftJoy / 127, 3) * 127;
        double right = pow(rightJoy / 127, 3) * 127;
        leftMotors.move(left);
        rightMotors.move(right);
        if(allowHang && r == 1 && y == 1) {
            allowHang = false;
            hang.set_value(true);
        } else if(r == 0 || y == 0) {
            allowHang = true;
        }
        if(allowWings && w == 1) {
            allowWings = false;
            wings.set_value(!wingState);
            wingState = !wingState;
        } 
        else if(w == 0) {
            allowWings = true;
        }

        if(h == 1) {
            intake.set_value(true);
        } else {
            intake.set_value(false);
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
