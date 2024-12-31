using namespace vex;

extern brain Brain;
extern controller Controller1;


extern motor intake;
extern motor LeftDTMotorA;
extern motor LeftDTMotorB;
extern motor LeftDTMotorC;
extern motor_group LeftDT;
extern motor RightDTMotorA;
extern motor RightDTMotorB;
extern motor RightDTMotorC;
extern motor_group RightDT;
extern motor ramp;
extern digital_out mogo_clamp;
extern inertial inertialSensor;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);