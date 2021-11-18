#include "main.h"

using namespace okapi;

extern Motor leftBack;
extern Motor leftFront;
extern Motor rightBack;
extern Motor rightFront;

void translatePID(double distance);

void rotatePID(double turnDegrees);

void jankRotatePID(double leftDistance, double rightDistance);
