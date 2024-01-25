#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "vex.h"
#include <iostream>

#define PI atan(1)*4
#define WHEEL_DIAMETER 2.9
#define PPR500 9/50
#define PPR5120 9/512

using namespace vex;

struct Vec
{
  double x = 0.0;
  double y = 0.0; 
  double z = 0.0;

  Vec(double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
};


class odometry
{
private:
  inertial *iner1 = nullptr;
  double errorAdj = 1.025;
  double L = 7.6;
  double B = 0.8;
  bool singleTacerWheel = false;
  Vec pos = Vec(0.0, 0.0, 0.0);

public:
  odometry() {}
  odometry(double _length, double _back) : L(_length), B(_back) {}
  odometry(double _length, double _back, inertial &useImu) : L(_length), B(_back), iner1(&useImu) {}
  odometry(inertial &useImu) : iner1(&useImu) { singleTacerWheel = true; }
  void updateRobotPosition();
  const Vec &getPosition();
  double getGoalDirection(Vec origin, unsigned int quadrant);
  double getDistanceFromGoal(Vec origin, unsigned int quadrant);
  void displayPosition();
  void setStartingPosition(const Vec starting_pos) { pos = Vec(starting_pos.x, starting_pos.y, starting_pos.z); }
  const double getRbtYPos();
  const double getRevYPos();
  double toDegrees(double radians);
  double toRadians(double degrees);
  double wrapAngle(double degrees);
  bool displayPositionOnScreen = true;
  bool isCalibrating();
};


void odometry::updateRobotPosition()
{
  double oldForwardRotation = 0.0;
  double oldLeftRotation = 0.0;
  double oldRightRotation = 0.0;
  double oldHorizontalRotation = 0.0;
  double oldHeading = 0.0;
  double currentForwardRotation = 0.0;
  double currentLeftRotation = 0.0;
  double currentRightRotation = 0.0;
  double currentHorizontalRotation = 0.0;
  double currentHeading = 0.0;

  while (true)
  {
    currentForwardRotation = -1.0 * ((leftFrontT.position(rev) + rightFrontT.position(rev)) / 2.0);
    currentHeading = (iner1 != nullptr) ? iner1->rotation(rotationUnits::deg) * (errorAdj) * ((PI) / 180) : 0;
    
    double dn = currentForwardRotation - oldForwardRotation;
    double dn1 = currentLeftRotation - oldLeftRotation;
    double dn2 = currentRightRotation - oldRightRotation;
    double dn3 = currentHorizontalRotation - oldHorizontalRotation;

    double dtheta = (iner1 != nullptr) ? currentHeading - oldHeading : ((((PI) * (WHEEL_DIAMETER)) * (dn1 - dn2)) / L);
    double dx = (singleTacerWheel) ? 0 : ((PI * WHEEL_DIAMETER) * (dn3 - (dn1 - dn2) * B / L));
    double dy = (singleTacerWheel) ? ((PI * WHEEL_DIAMETER) * dn) : ((PI * WHEEL_DIAMETER) * (dn1 + dn2) / 2.0);

    double theta = pos.z + (dtheta / 2.0);
    pos.x += (dx * cos(theta) + dy * sin(theta));
    pos.y += (dy * cos(theta) - dx * sin(theta));
    pos.z += dtheta;

    if (displayPositionOnScreen)
    {
      displayPosition();
    }

    oldForwardRotation = currentForwardRotation;
    oldLeftRotation = currentLeftRotation;
    oldRightRotation = currentRightRotation;
    oldHorizontalRotation = currentHorizontalRotation;
    oldHeading = currentHeading;

    wait(10, msec);
  }
}

double odometry::toDegrees(double radians)
{
  return radians * (180 / (PI));
}

double odometry::toRadians(double degrees)
{
  return degrees * ((PI) / 180);
}

double odometry::wrapAngle(double degrees)
{
    while (degrees > 180)
    {
      degrees -= 360;
    }
    while (degrees < -180)
    {
      degrees += 360;
    }
    return degrees;
}

const double odometry::getRbtYPos()
{
  return ((PI * WHEEL_DIAMETER) * 1.0 * ((leftFrontT.position(rev) + rightFrontT.position(rev)) / 2.0));
}

const double odometry::getRevYPos()
{
    return ((PI * WHEEL_DIAMETER) * -1.0 * ((leftFrontT.position(rev) + rightFrontT.position(rev)) / 2.0));
}

double odometry::getGoalDirection(Vec origin, unsigned int quadrant)
{
    Vec goalPosition = (quadrant == 2 || quadrant == 4) ? Vec(130, 130, 0) : Vec(18, 120, 0);
    double x = goalPosition.x - origin.x - pos.x;
    double y = goalPosition.y - origin.y - pos.y;
    return wrapAngle(toDegrees(atan2(x, y) - pos.z));
}

double odometry::getDistanceFromGoal(Vec origin, unsigned int quadrant)
{
    Vec goalPosition = (quadrant == 2 || quadrant == 4) ? Vec(130, 130, 0) : Vec(24, 120, 0);
    double x = goalPosition.x - origin.x - pos.x;
    double y = goalPosition.y - origin.y - pos.y;
    return hypot(x, y);
}

bool odometry::isCalibrating()
{
    return (iner1 != nullptr && iner1->isCalibrating());
}

void odometry::displayPosition()
{
  Brain.Screen.setPenColor(vex::color(170, 170, 170));
  Brain.Screen.setCursor(9, 27);
  Brain.Screen.print(pos.x);
  Brain.Screen.setCursor(10, 27);
  Brain.Screen.print(pos.y);
  if (iner1 == nullptr || (iner1 != nullptr && !iner1->isCalibrating()))
  {
    Brain.Screen.setCursor(11, 27);
    Brain.Screen.print(toDegrees(pos.z));
  }
}

const Vec &odometry::getPosition()
{
    return pos;
}

#endif ODOMETRY_H