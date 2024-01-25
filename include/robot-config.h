using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftFrontT;
extern motor leftFrontB;
extern motor leftBackB;
extern motor leftBackT;
extern motor rightBackT;
extern motor rightBackB;
extern motor rightFrontB;
extern motor rightFrontT;
extern motor Intake;
extern optical OPTICAL;
extern inertial INERTIAL;
extern motor Cata1;
extern motor Cata2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );