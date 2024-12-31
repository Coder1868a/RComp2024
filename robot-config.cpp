#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller1 = controller(primary);
/////////////
motor intake = motor(PORT9, ratio6_1, true);
motor LeftDTMotorA = motor(PORT18, ratio6_1, true);
motor LeftDTMotorB = motor(PORT19, ratio6_1, true);
motor LeftDTMotorC = motor(PORT20, ratio6_1, true); // 12
motor_group LeftDT = motor_group(LeftDTMotorA, LeftDTMotorB, LeftDTMotorC);
motor RightDTMotorA = motor(PORT13, ratio6_1, false);
motor RightDTMotorB = motor(PORT12, ratio6_1, false);
motor RightDTMotorC = motor(PORT11, ratio6_1, false);
motor ramp = motor(PORT10, ratio6_1, false);
motor_group RightDT = motor_group(RightDTMotorA, RightDTMotorB, RightDTMotorC);

digital_out mogo_clamp = digital_out(Brain.ThreeWirePort.H);
inertial inertialSensor = inertial(PORT14);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  
}