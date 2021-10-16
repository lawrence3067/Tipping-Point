#include "main.h"

using namespace okapi;

extern Motor leftBack;
extern Motor leftFront;
extern Motor rightBack;
extern Motor rightFront;
extern ADIEncoder encoder;

void updateDrive();

//void odometry(double x_pos, double y_pos);

void translatePID(double leftDistance, double rightDistance);

void rotate_PID(double turn_degrees);
