#include "main.h"

using namespace okapi;

extern Motor rightFront;
extern Motor rightTop;
extern Motor rightBottom;

extern Motor leftFront;
extern Motor leftTop;
extern Motor leftBottom;

extern IMU inertialSensor;
extern IMU inertialSensor2;

extern std::shared_ptr<ChassisController> drive;

void updateDrive();

void translatePID(double targetDistance, int ms, bool clamp);
void rotatePID(double targetAngle, int ms);
void translateBitch(double targetDistance, int ms, bool clamp);
