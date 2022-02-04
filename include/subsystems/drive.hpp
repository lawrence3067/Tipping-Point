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

extern std::shared_ptr<OdomChassisController> drive;

void updateDrive();
void rotatePID(double angle, int ms);
void translatePID(double distance, int ms);
void autonPark();
