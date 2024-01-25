#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftFrontT = motor(PORT11, ratio6_1, false);
motor leftFrontB = motor(PORT12, ratio6_1, true);
motor leftBackB = motor(PORT13, ratio6_1, true);
motor leftBackT = motor(PORT14, ratio6_1, false);
motor rightBackT = motor(PORT16, ratio6_1, true);
motor rightBackB = motor(PORT17, ratio6_1, false);
motor rightFrontB = motor(PORT18, ratio6_1, false);
motor rightFrontT = motor(PORT19, ratio6_1, true);
motor Intake = motor(PORT20, ratio6_1, true);
optical OPTICAL = optical(PORT10);
inertial INERTIAL = inertial(PORT15);
motor Cata1 = motor(PORT6, ratio36_1, false);
motor Cata2 = motor(PORT7, ratio36_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}