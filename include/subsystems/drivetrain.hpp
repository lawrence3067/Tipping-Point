#include "main.h"

using namespace okapi;

extern Motor rightFront;
extern Motor rightTop;
extern Motor rightBottom;

extern Motor leftFront;
extern Motor leftTop;
extern Motor leftBottom;
extern QVector<QLength> pos; // the current position of the robot
extern QVector<Number> heading; // the "heading" or direction of the robot (this is always a unit vector)
extern QAngle curAngle; // the current direction of the robot, but as an angle. just to make the actual angle easier to access
extern double rPrev; // previous encoder value for the right encoder wheel
extern double lPrev;
void updateDrive();
void rotatePID(double angle, int ms);
void translatePID(double distance, int ms);
void update();
