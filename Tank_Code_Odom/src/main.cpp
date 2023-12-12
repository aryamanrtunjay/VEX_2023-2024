// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         1               
// FR                   motor         20              
// ML                   motor         9               
// MR                   motor         17              
// BL                   motor         15              
// BR                   motor         5               
// Controller1          controller                    
// Arm                  motor         8               
// Flytake              motor         18              
// Wing                 digital_out   A               
// Inertial             inertial      12              
// PTO                  digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "iostream"
#include "vector"

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
  FL.setPosition(0, degrees);
  ML.setPosition(0, degrees);
  BL.setPosition(0, degrees);
  FR.setPosition(0, degrees);
  MR.setPosition(0, degrees);
  BR.setPosition(0, degrees);
  Flytake.setPosition(0, degrees);
  Arm.setPosition(0, degrees);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//  --- Auton Coords ---
double xCoords[53] = {
35.73267716535433,
35.75275590551181,
35.80826771653543,
35.90551181102362,
36.0496062992126,
36.241732283464565,
36.48464566929134,
36.779921259842524,
37.128740157480316,
37.52913385826771,
37.97874015748031,
38.47322834645669,
39.008661417322834,
39.57992125984252,
40.18149606299213,
40.80866141732283,
41.45669291338582,
42.12204724409449,
42.801574803149606,
43.49212598425197,
44.19173228346457,
44.89803149606299,
45.60984251968504,
46.3251968503937,
47.042519685039366,
47.76102362204725,
48.47952755905512,
49.196850393700785,
49.911417322834644,
50.62204724409449,
51.327559055118115,
52.025984251968495,
52.71535433070866,
53.39370078740158,
54.05826771653543,
54.70669291338583,
55.33464566929134,
55.938976377952756,
56.51535433070866,
57.059842519685034,
57.56771653543307,
58.035039370078735,
58.45826771653544,
58.83503937007874,
59.16377952755906,
59.444881889763785,
59.67874015748031,
59.86732283464566,
60.010236220472436,
60.11456692913386,
60.18070866141733,
60.216929133858265,
60.216929133858265,
};

double yCoords[53] = {
-60.90314960629921,
-60.11614173228346,
-59.331102362204724,
-58.5496062992126,
-57.775590551181104,
-57.01220472440945,
-56.263385826771646,
-55.53385826771654,
-54.82834645669292,
-54.150393700787404,
-53.50433070866142,
-52.89212598425197,
-52.31496062992126,
-51.77322834645669,
-51.26535433070866,
-50.789370078740156,
-50.34251968503937,
-49.92165354330709,
-49.523622047244096,
-49.14566929133858,
-48.78425196850394,
-48.436220472440944,
-48.0992125984252,
-47.77007874015748,
-47.44566929133858,
-47.12401574803149,
-46.801574803149606,
-46.47677165354331,
-46.146062992125984,
-45.80708661417322,
-45.45708661417323,
-45.093307086614175,
-44.71299212598425,
-44.31338582677165,
-43.891338582677164,
-43.44448818897638,
-42.96968503937008,
-42.46535433070866,
-41.92913385826772,
-41.360236220472444,
-40.75905511811024,
-40.1255905511811,
-39.46181102362205,
-38.770472440944886,
-38.05551181102362,
-37.320078740157484,
-36.568503937007875,
-35.80393700787401,
-35.02992125984252,
-34.2496062992126,
-33.46496062992126,
-32.3003937007874,
-32.3003937007874,
};

double vels[53] = {
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
120.0,
0.0,
};                             

//  --- General Fields ---
double pi = 3.14159265;
double degToInch = (1.0/360) * pi * 3.25 * (3.0/4.0);
bool resetDriveEncoders = false;
double matchTime = 0;

//  --- PID Fields ---
bool enableDrivePID = true;

// Settings
// double kU = 0.30;
// double pU = 0.70;
// double kP = 0.6 * kU;
// double kI = 0.3 * kU / pU;
// double kD = 0.075 * kU / pU;

double kP = 0.068;
double kI = 0.0;
double kD = 0;

double K_v = 1 / (400 * 360 * degToInch);
double K_a = 0;
double K_p = 0;

double turnkP = 6;
double turnkI = 0.00001;
double turnkD = 0.4;

// Autonomous Settings
double desiredValue = 0;
int desiredTurnValue = 0;

int error; // SensorVal - TargetVal : Positional Value
int prevError = 0; // Position 20 ms ago
int derivative; // error - prevError : Speed
int totalError = 0; // Integral : Absement
int turnError;
int turnPrevError = 0;
int turnDerivative;
int turnTotalError = 0;

double cX = 35.733;
double cY = -60.903;

double prevLH = 0;

int odometry() {
  double prevDistance = 0;
  while(true) {
    double leftMotorPositionDeg = (ML.position(degrees) + BL.position(degrees) + FL.position(degrees)) / 3;
    double rightMotorPositionDeg = (MR.position(degrees) + BR.position(degrees) + FR.position(degrees)) / 3;
    double distance = (leftMotorPositionDeg + rightMotorPositionDeg) / 2;
    double dDist = distance - prevDistance;
    dDist = dDist * degToInch;
    double dX = dDist * cos(Inertial.heading());
    double dY = dDist * sin(Inertial.heading());

    cX += dX;
    cY += dY;


    prevDistance = distance;
    vex::task::sleep(2);
  }

  return 1;
}

double dot(vector<double> v1, vector<double> v2) {
  return v1[0] * v2[0] + v1[1] * v2[1];
}

vector<double> get_lookahead(double r) {
  for(int i = prevLH; i < sizeof(xCoords) - 1; i++) {
    vector<double> E = {xCoords[i], yCoords[i]};
    vector<double> L = {xCoords[i + 1], yCoords[i + 1]};
    vector<double> C = {cX, cY};

    vector<double> d = {L[0] - E[0], L[1] - E[1]};
    vector<double> f = {L[0] - E[0], L[1] - E[1]};

    double a = dot(d, d);
    double b = 2 * dot(f, d);
    double c = dot(f, f) - r * r;
    double discriminant = b * b - 4 * a * c;

    if(discriminant > 0) {
      discriminant = sqrt(discriminant);
      double t1 = (-b - discriminant) / (2 * a);
      double t2 = (-b + discriminant) / (2 * a);

      if(t1 >= 0 && t1 <= 1) {
        vector<double> out = {E[0] + t1 * d[0], E[1] + t1 * d[1]};
        return out;
      }
      if(t2 >= 0 && t2 <= 1) {
        vector<double> out = {E[0] + t2 * d[0], E[1] + t2 * d[1]};
      }
    }
    prevLH = i;
  }
  return {0, 0};
}

int pure_pursuit() {
  double minDist = 9999999;
  double minIdx = 0;

  double closeX = 0;
  double closeY = 0;

  for(int i = 0; i < sizeof(xCoords); i++) {
    double x = xCoords[i];
    double y = yCoords[i];

    double dist = sqrt(pow(x - cX, 2) + pow(y - cY, 2));
    if(dist < minDist) {
      dist = minDist;
      minIdx = i;
      closeX = x;
      closeY = y;
    }

    vector<double> lh = get_lookahead(12);
    double lX = lh[0];
    double lY = lh[1];

    double xDist = lX - cX;
    double yDist = lY - cY;
    double L = sqrt(pow(xDist, 2) + pow(yDist, 2));
    double curvature = 2 * x / pow(L, 2);
    double trackWidth = 17;
    double targetVelocity = vels[i];
    double prevLeft = 0;
    double prevRight = 0;
    double leftSpeed = targetVelocity * (2 + curvature * trackWidth) / 2;
    double rightSpeed = targetVelocity * (2 - curvature * trackWidth) / 2;
    double FF_Left = K_v * leftSpeed + K_a * (leftSpeed - prevLeft);
    double FF_Right = K_v * rightSpeed + K_a * (rightSpeed - prevRight);

    double leftMotorVel = (FL.velocity(dps) + ML.velocity(dps) + BL.velocity(dps))/3;
    double rightMotorVel = (FR.velocity(dps) + MR.velocity(dps) + BR.velocity(dps))/3;
    double act_Vel_L = leftMotorVel * degToInch;
    double act_Vel_R = rightMotorVel * degToInch;

    double FB_Left = K_p * (leftSpeed - act_Vel_L);
    double FB_Right = K_p * (rightSpeed - act_Vel_R);

    cout << targetVelocity  << endl;
    
    FL.spin(vex::directionType::fwd, (FF_Left + FB_Left) / degToInch, vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, (FF_Left + FB_Left) / degToInch, vex::velocityUnits::dps);
    MR.spin(vex::directionType::fwd, (FF_Left + FB_Left) / degToInch, vex::velocityUnits::dps);
    BL.spin(vex::directionType::fwd, (FF_Left + FB_Left) / degToInch, vex::velocityUnits::dps);
    BR.spin(vex::directionType::fwd, (FF_Right + FB_Right) / degToInch, vex::velocityUnits::dps);
    FR.spin(vex::directionType::fwd, (FF_Right + FB_Right) / degToInch, vex::velocityUnits::dps);

    prevLeft = leftSpeed;
    prevRight = rightSpeed;
    if(i == sizeof(xCoords) - 1) { 
      i = 0;
    }
    vex::task::sleep(2);
  }

  return 1;
}

int drivePID() {
  while(enableDrivePID) {
    if(resetDriveEncoders) {
      resetDriveEncoders = false;

      FL.setPosition(0, degrees);
      ML.setPosition(0, degrees);
      BL.setPosition(0, degrees);
      FR.setPosition(0, degrees);
      MR.setPosition(0, degrees);
      BR.setPosition(0, degrees);
    }
    // -------------------------- Lateral Movement PID -------------------------- //

    int leftMotorPositionDeg = (ML.position(degrees) + BL.position(degrees) + FL.position(degrees));
    int rightMotorPositionDeg = (MR.position(degrees) + BR.position(degrees) + FR.position(degrees));
    int averagePosition = (leftMotorPositionDeg + rightMotorPositionDeg) / 6;
    // int averagePosition = averagePositionDeg / degToInch;

    error = desiredValue - averagePosition;
    derivative = error - prevError;
    totalError += error;

    double latMotorPower = ((error * kP) + (totalError * kI) + (derivative * kD)); 
    // -------------------------- Lateral Movement PID -------------------------- //

    // -------------------------- Turning Movement PID -------------------------- //
    double h = Inertial.heading(degrees);
    if(h >= 180) h = h - 360;
    // cout << h << endl;
    turnError = h - desiredTurnValue;

    turnDerivative = turnError - turnPrevError;
    turnTotalError += turnError;

    double turnMotorPower = ((turnError * turnkP) + (turnTotalError * turnkI) + (turnDerivative * turnkD)) / 12.0; 
    if(desiredTurnValue >= 360) turnMotorPower = 0;
    // -------------------------- Turning Movement PID -------------------------- //
    double speedLimit = 0.5;
    FL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    ML.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    BL.spin(vex::directionType::fwd, (latMotorPower - turnMotorPower), vex::velocityUnits::pct);
    MR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    BR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    FR.spin(vex::directionType::fwd, (latMotorPower + turnMotorPower), vex::velocityUnits::pct);
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(2);
  }

  return 1;
}

void drive(double inches, double delay) {
  vex::task::sleep(delay);
  resetDriveEncoders = true;
  desiredValue = inches / degToInch;
}

void turn(double deg, double delay) {
  vex::task::sleep(delay);
  resetDriveEncoders = true;
  desiredValue = 0;
  desiredTurnValue = deg;
}

void arc(double d, double a, double delay) {
  resetDriveEncoders = true;
  desiredValue = d;
  desiredTurnValue = a;
  vex::task::sleep(delay);
}

void autonomous(void) {
  vex::task runDrivePID(drivePID);

  arc(27, -90, 3000);
}


void usercontrol(void) {
  enableDrivePID = false;
  bool openRWing = false;
  bool openLWing = false;
  bool allowA = true;
  bool allowLeft = true;
  bool isBall = false;
  bool armUp = false;
  bool moveArmUp = false;
  bool moveArmDown = true;

  double above1 = 0;
  // Driver Control Main Loop
  while (1) {
    int leftStick = Controller1.Axis3.value();
    int rightStick = Controller1.Axis2.value();
    bool L1 = Controller1.ButtonL1.pressing();
    bool L2 = Controller1.ButtonL2.pressing();
    bool R2 = Controller1.ButtonR2.pressing();
    bool R1 = Controller1.ButtonR1.pressing();
    bool A = Controller1.ButtonA.pressing();
    bool left = Controller1.ButtonLeft.pressing();
    int stickDeadZone = 5;
    bool outtake = false;

    if(leftStick > 0) {
      leftStick = (leftStick * leftStick) / 200;
    } else if(leftStick < 0) {
      leftStick = (leftStick * leftStick) / -200;
    }

   if(rightStick > 0) {
      rightStick = (rightStick * rightStick) / 200;
    } else if(leftStick < 0) {
      rightStick = (rightStick * rightStick) / -200;
    }

    // Left wheel control
    if(abs(leftStick) > stickDeadZone) { // Check dead zone
      FL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      ML.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
      BL.spin(vex::directionType::fwd, leftStick, vex::velocityUnits::pct);
    }
    else {
      FL.stop(coast);
      ML.stop(coast);
      BL.stop(coast);
    }

    // Right wheel control
    if(abs(rightStick) > stickDeadZone) {
      FR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      MR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
      BR.spin(vex::directionType::fwd, rightStick, vex::velocityUnits::pct);
    }
    else {
      FR.stop(coast);
      MR.stop(coast);
      BR.stop(coast);
    }

    if(L1 && !armUp) {
      moveArmUp = true;
    }
    if(L2) {
      if(armUp) {
        moveArmDown = true;
      }
      else { 
        Flytake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      }
    }
    
    if(L1 && moveArmDown) {
      moveArmDown = false;
      moveArmUp = true;
    }
    if(L2 && moveArmUp) {
      moveArmDown = true;
      moveArmUp = false;
    }

    if(!L2 && !armUp) {
      Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }

    if(moveArmUp && Inertial.pitch() < 70) {
      Arm.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      PTO.set(true);
      Flytake.spin(vex::directionType::fwd, 60, vex::velocityUnits::pct);
    }
    else if(moveArmUp) {
      moveArmUp = false;
      Arm.stop(coast);
      armUp = true;
    }
    if(moveArmDown && Inertial.pitch() > 28) {
      PTO.set(false);
      Arm.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      Flytake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }
    else if(moveArmDown) {
      moveArmDown = false;
      Arm.stop(coast);
      armUp = false;
    }
 
    if(R2 && allowA) {
      allowA = false;
      if(openRWing) {
        Wing.set(true);
      }
      else {
        Wing.set(false);
      }
      openRWing = !openRWing;
    }
    else if(!R2) {
      allowA = true;
    }

    matchTime += 20;
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  // Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}