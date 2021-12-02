#include "main.h"

using namespace okapi;

extern Motor leftBack;
extern Motor leftFront;
extern Motor rightBack;
extern Motor rightFront;

void translatePID(double distance, int ms);

void rotatePID(double turnDegrees, int ms);

void jankRotatePID(double leftDistance, double rightDistance);
