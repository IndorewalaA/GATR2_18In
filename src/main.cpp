/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: Abdul (Auton), Joseph(Driver) */
/* Created: 1/19/2024 */
/* Description: Version1-Asburn */
/* */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftFrontT           motor         11              
// leftFrontB           motor         12              
// leftBackB            motor         13              
// leftBackT            motor         14              
// rightBackT           motor         16              
// rightBackB           motor         17              
// rightFrontB          motor         18              
// rightFrontT          motor         19              
// Intake               motor         20              
// OPTICAL              optical       10              
// INERTIAL             inertial      15              
// Cata1                motor         6               
// Cata2                motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "odometry.h"
#include "DriverCurve.h"

using namespace vex;

odometry Odom;
motor_group leftMG (leftFrontB, leftBackB, leftFrontT, leftBackT);
motor_group rightMG (rightFrontB, rightBackB, rightFrontT, rightBackT);

// A global instance of competition
competition Competition;

driverCurve testRemote;

int forwardPID(double target, double P, double I, double D, bool fwrd)
{
  double totErr = 0;
  double lastErr = Odom.getRbtYPos();
  while (true)
  {
    double currVal = Odom.getRbtYPos();
    double currErr = target - currVal;
    totErr += currErr;
    double diffErr = currErr - lastErr;

    double currVolt = currErr*P + totErr*I + diffErr*D;

    if (fwrd)
    {
      leftMG.spin(forward, currVolt, voltageUnits::volt);
      rightMG.spin(forward, currVolt, voltageUnits::volt);
    }
    else
    {
      leftMG.spin(reverse, currVolt, voltageUnits::volt);
      rightMG.spin(reverse, currVolt, voltageUnits::volt);
    }
    lastErr = currErr;
    //std::cout << "f" << currErr << std::endl;
    if (currErr < 1)
    {
      leftMG.stop(coast);
      rightMG.stop(coast);
      std::cout << "f" << currErr << std::endl;
      break;
    }
  }
  return 1;
}

int reversePID (double target, double P, double I, double D, bool rev)
{
  double totErr = 0;
  double lastErr = Odom.getRevYPos();
  while (true)
  {
    double currVal = Odom.getRevYPos();
    double currErr = target - currVal; 
    totErr += currErr;
    double diffErr = currErr - lastErr;

    double currVolt = currErr*P + totErr*I + diffErr*D;
    
    if (rev)
    {
      leftMG.spin(reverse, currVolt, voltageUnits::volt);
      rightMG.spin(reverse, currVolt, voltageUnits::volt);
    }
    else
    {
      leftMG.spin(reverse, currVolt, voltageUnits::volt);
      rightMG.spin(reverse, currVolt, voltageUnits::volt);
    }
    lastErr = currErr;

    vex::task::sleep(20);

    if (currErr < 1)
    {
      leftMG.stop(coast);
      rightMG.stop(coast);
      std::cout << "r" << currErr << std::endl;
      break;
    }
  }
  return 1;
}

int rotatingPID(double turnTarget, double tP, double tI, double tD)
{
  double totErr = 0;
  double lastErr = Odom.toDegrees(Odom.getPosition().z);
  while (true)
  {
    double currVal = Odom.toDegrees(Odom.getPosition().z);
    double currErr = turnTarget - currVal;
    totErr += currErr;
    double diffErr = currErr - lastErr;

    double currVolt = currErr*tP + totErr*tI + diffErr*tD;

    leftMG.spin(forward, currVolt, voltageUnits::volt);
    rightMG.spin(forward, currVolt, voltageUnits::volt);

    lastErr = currErr;
  
    vex::task::sleep(20);
    Brain.Screen.printAt(1, 20, "PID Voltage: ", currVolt);
    printf( " Current Error %f\n", currErr);

    if (currErr < 2)
    {
      leftMG.stop(coast);
      rightMG.stop(coast);
      break;
    }
  }
  return 1;
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  leftMG.resetPosition();
  rightMG.resetPosition();

  Odom = odometry(INERTIAL);

  vex::thread ([]()
  {
    while(true)
    {
      Odom.updateRobotPosition();
    }
  });
  forwardPID(60, 0.5, 0.0, 0.0, true);
  rightMG.resetPosition();
  leftMG.resetPosition();
  reversePID(-60, 0.5, 0.0, 0.0, true);
  rightMG.resetPosition();
  leftMG.resetPosition();
}


void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    if( testRemote.rightAxisPos() > 10 || testRemote.rightAxisPos() < -10){
      rightMG.spin(forward, testRemote.rightCurve() , volt);
    }
    else{
      rightMG.stop();
    }
    if(testRemote.leftAxisPos() > 10 || testRemote.leftAxisPos() < -10){
      leftMG.spin(forward, testRemote.leftCurve(), volt);
    }
    else{
      leftMG.stop();
    }

    if(Controller1.ButtonL2.pressing() && Controller1.ButtonR2.pressing()){
      rightMG.spin(forward, -1, volt);
      leftMG.spin(forward, -1, volt);
      leftMG.stop();
      rightMG.stop();
    }

    if(Controller1.ButtonX.pressing()){
      Intake.spin(forward, 12, volt); 
    }
    if(Controller1.ButtonA.pressing()){
      Intake.stop(); 
    }
    if(Controller1.ButtonB.pressing()){
      Intake.spin(forward, -12, volt); 
    }
//Catapult
    if(Controller1.ButtonR1.pressing()){
      Cata1.spin(forward, 12, volt);
      Cata2.spin(forward, -12, volt); 
    }
    if(Controller1.ButtonL1.pressing()){
      Cata1.stop(); 
      Cata2.stop(); 
    }
    wait(20, msec);
  }
}



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
