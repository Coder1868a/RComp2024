/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       juliasomlo                                                */
/*    Created:      Thu Dec 05 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
bool isPIDon = false; // well drivePID on?

int deadband = 3;



void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertialSensor.resetRotation();
  inertialSensor.calibrate();
  // while(inertialSensor.isCalibrating()){
  //   Brain.Screen.setCursor(11, 1);
  //   Brain.Screen.print("inertial calibrating");
  //   for(int n = 0; n > 20000; n+= 10){
  //     vex::task::sleep(10);
  //     Brain.Screen.setCursor(10, 0);
  //     Brain.Screen.print("Calibration Time = %f", n);
  //   }
    
  // }

  mogo_clamp.set(false);
  intake.setVelocity(320, rpm); // 230
  LeftDT.setVelocity(220,rpm);
  RightDT.setVelocity(220,rpm); //210
  ramp.setVelocity(290, rpm); // 220
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

float error = 0;
float prevError = 0;
float turnerror = 0;
float prevturnerror;
float turndifference;
float averageposition;
float totalerror = 0;
float totalturnerror = 0;
float desiredvalue = 200;
float desiredturnvalue = 0;
// tuning values
long double kI = 0;
long double kD = 0.008;
long double kP = 0.0000003;
long double tP = 0.0000000106;
long double tI = 0.00005;
long double tD = 0.000018;

float GEAR_RATIO = 0.75;
bool resetDriveSensors = false;
float derivative; 
float turnderivative;
float integral;
float turnintegral;
double turnmotorpower;
double lateralmotorpower;



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// int drivePID(){
//   if((resetDriveSensors) == true){
//       LeftDT.setPosition(0, degrees);
//       RightDT.setPosition(0,degrees);
//       inertialSensor.resetRotation();
//    }
//   while(isPIDon){

    
//     int leftmotorposition = LeftDT.position(degrees);
//     int rightmotorposition = RightDT.position(degrees);
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print("leftmotorposition = %f", leftmotorposition);
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.print("rightmotorposition = %f", rightmotorposition);

//     int averageposition = (leftmotorposition + rightmotorposition)/2;

//     error = desiredvalue - averageposition;

    

//     derivative = error - prevError;

//     totalerror += error;

//     lateralmotorpower = (error * kP + totalerror * kI + derivative * kD);
//     Brain.Screen.setCursor(3,1);
//     Brain.Screen.print("lateralmotorpower = %f", lateralmotorpower);
//     // Turning PID
//     turnerror = desiredturnvalue - inertialSensor.rotation(deg);
//     totalturnerror += turnerror;
//     turnmotorpower = (turnerror * tP + totalturnerror * tI + turnderivative * tD);
//     Brain.Screen.setCursor(4,1);
//     Brain.Screen.print("turnmotorpower = %f", turnmotorpower);

//     // spin forever 
//     LeftDT.spin(forward, lateralmotorpower - turnmotorpower,voltageUnits::volt);
//     RightDT.spin(forward, lateralmotorpower + turnmotorpower, voltageUnits::volt);
//     vex::task::sleep(20);

//     Brain.Screen.setCursor(5,1);
//     Brain.Screen.print("desiredvalue = %f", desiredvalue);

//     Brain.Screen.setCursor(6,1);
//     Brain.Screen.print("desiredturnvalue = %f", desiredturnvalue);

//     Brain.Screen.setCursor(7,1);
//     Brain.Screen.print("error = %f", error);

//     Brain.Screen.setCursor(8,1);
//     Brain.Screen.print("totalerror = %f", totalerror);
    
//     Brain.Screen.setCursor(9,1);
//     Brain.Screen.print("averageposition = %f", averageposition);

//     Brain.Screen.setCursor(10,1);
//     Brain.Screen.print("derivative = %f",derivative);

//     // end of code
//     prevError = error;
//     prevturnerror = turnerror;
    
//   }
//   return 1;
// }

int expoTurning(int joystickVal) {
  int retVal = 0;
  if (abs(joystickVal) > deadband) {
    int direction = abs(joystickVal) / joystickVal;
    retVal = direction * (0.7 * pow(1.0387, abs(joystickVal)) - 0.7 + 0.096 * abs(joystickVal));
  }
  return retVal;
}
float WHEEL_DIAM = 3.25;

float pi = 3.1415;
float inchtodeg;
float inchestodegrees(float inches){
  inchtodeg = (inches * 360* GEAR_RATIO)/(2*pi*WHEEL_DIAM);
  return inchtodeg;
}

void toggle_clamp(){
  mogo_clamp.set(!mogo_clamp.value());
  wait(200, msec);
}

// double hue = colorSensor.hue();

void autonomous(void) {

  LeftDT.spinFor(reverse, inchestodegrees(63), degrees, false);
  RightDT.spinFor(reverse, inchestodegrees(63), degrees);

  LeftDT.setVelocity(100,rpm);
  RightDT.setVelocity(100,rpm);

  vex::task::sleep(500);
  
  LeftDT.spinFor(reverse, inchestodegrees(17), degrees, false);
  RightDT.spinFor(reverse, inchestodegrees(17), degrees, false);

  vex::task::sleep(1000);

  toggle_clamp();

  ramp.spin(forward);


  vex::task::sleep(2000);

  toggle_clamp();


  LeftDT.spinFor(reverse, inchestodegrees(-24), degrees, false);
  RightDT.spinFor(reverse, inchestodegrees(-24), degrees);

  vex::task::sleep(200);

  LeftDT.spinFor(reverse, inchestodegrees(11), degrees, false);
  RightDT.spinFor(forward, inchestodegrees(11), degrees);


  vex::task::sleep(200);

  LeftDT.spinFor(reverse, inchestodegrees(70), degrees, false);
  RightDT.spinFor(reverse, inchestodegrees(70), degrees);

  

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}




/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  ramp.stop();
  isPIDon = false;
  while (1) {
    int leftmotorspeed = 1.7*(expoTurning(Controller1.Axis3.position())) + (expoTurning(Controller1.Axis1.position()));
    int rightmotorspeed = 1.7*(expoTurning(Controller1.Axis3.position())) - (expoTurning(Controller1.Axis1.position()));

    if(abs(leftmotorspeed) < deadband){
      LeftDT.setVelocity(0, percent);
    } else{
      LeftDT.setVelocity(leftmotorspeed, percent);
    }
      if(abs(rightmotorspeed) < deadband){
      RightDT.setVelocity(0, percent);
    } else{
      RightDT.setVelocity(rightmotorspeed, percent);
    }

    if(Controller1.ButtonR1.pressing()){
      ramp.stop();
    } 
    if(Controller1.ButtonR2.pressing()){
      ramp.spin(forward);

    }
    if(Controller1.ButtonL1.pressing()){
      intake.stop();
    }
    if(Controller1.ButtonL2.pressing()){
      intake.spin(forward);
    }
    if(Controller1.ButtonDown.pressing()){
      intake.spin(reverse);
    }
    if(Controller1.ButtonB.pressing()){
      ramp.spin(reverse);
    }

    LeftDT.spin(forward);
    RightDT.spin(forward);

    Controller1.ButtonUp.pressed(toggle_clamp);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}