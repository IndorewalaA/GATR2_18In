/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: JOSEPH */
/* Created: 10/22/23 */
/* Description: Competition Template */
/* */
/*----------------------------------------------------------------------------*/

#ifndef DRIVER_H
#define DRIVER_H

#include "vex.h"
#include <iostream>

using namespace vex;

// the constructors have the same name as the class for its use
class driverCurve {
public:
driverCurve() {}
const double rightCurve();
const double leftCurve();

const double rightAxisPos();
const double leftAxisPos();
};

//For the foward axis
const double driverCurve::rightCurve() {
double rightJoyStick = Controller1.Axis2.position(percent)*1.0;
return (1.2*rightJoyStick*rightJoyStick*rightJoyStick)/100000;
}
const double driverCurve::leftCurve() {
double leftJoyStick = Controller1.Axis3.position(percent)*1.0;
return (1.2*leftJoyStick*leftJoyStick*leftJoyStick)/100000;
}


const double driverCurve::rightAxisPos() {
return Controller1.Axis2.position(percent)*1.0;

}
const double driverCurve::leftAxisPos() {
return Controller1.Axis3.position(percent)*1.0;
}


#endif DRIVER_H