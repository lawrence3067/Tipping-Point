#include "main.h"

using namespace okapi;

namespace odom
{
  extern QVector<QLength> pos;
  extern QVector<Number> heading;
  extern QAngle curAngle;
  extern double rPrev;
  extern double lPrev;

  extern void updateOdom();
  extern void driveToPoint(Point target, bool driveBack);
  extern void odomRotate(QAngle targetAngle);
  extern void odomTranslate(QLength targetDistance, bool driveBack);
  extern void odomDrift();
}

