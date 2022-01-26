#include "main.h"
#include <cmath>

using namespace okapi;
using namespace okapi::literals;

QAngle curAngle = 90_deg;
QVector<QLength> pos;
QVector<Number> heading;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightTop(rightTopPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBottom(rightBottomPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftTop(leftTopPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBottom(leftBottomPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

IMU inertialSensor(inertialPort, IMUAxes::z);


typedef struct PID pid;
pid translate;

int timer;
double inertial_values;
double turnError;
double threshold;
double integral;
double derivative;
double prevError;
double kP;
double ki;
double kd;
double p;
double i;
double d;
double vel;
double addValue;

std::shared_ptr<OdomChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftTop, leftBottom}, {rightFront, rightTop, rightBottom})
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 13.7_in}, imev5BlueTPR})
  .withOdometry()
  .buildOdometry();

void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
  if (controller.getDigital(ControllerDigital::left) == 1)
  {
    leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

    rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
    rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);
  }
  else if (controller.getDigital(ControllerDigital::right) == 1)
  {
    leftFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftTop.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftBottom.setBrakeMode(AbstractMotor::brakeMode::coast);

    rightFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightTop.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightBottom.setBrakeMode(AbstractMotor::brakeMode::coast);
  }
}

void rotatePID(double angle, int ms)
{
  inertialSensor.reset();
  timer = 0;
  threshold = 3;
  kP = 0.015;
  ki = 0;
  kd = 0.003;

  while (timer < ms)
  {
    turnError = angle - inertialSensor.get();
    integral = integral + turnError;
    derivative = turnError - prevError;
    prevError = turnError;
    p = turnError * kP;
    i = integral * ki;
    d = derivative * kd;

    vel = p + i + d;
    drive -> getModel() -> tank(vel, -vel);

    if(abs(turnError) < threshold && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
      && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftTop.getActualVelocity() < 1
      && leftBottom.getActualVelocity() < 1)
    {
      break;
    }

    timer += 10;
    pros::delay(10);
  }
  drive -> getModel() -> tank(0, 0);
}

void translatePID(double distance, int ms)
{
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	translate.target = distance * (360 / (2 * 3.1415 * (4 / 2)));

	//PID constants
	translate.kP = 0.002;
	translate.kI = 0;
	translate.kD = 0;

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);
	drive -> getModel() -> resetSensors();

  timer = 0;
  inertialSensor.reset();

	while (timer < ms)
	{
		translate.error = translate.target - ((drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2 * 18 / 35);
		translate.power = translateController.step(translate.error);

    drive -> getModel() -> tank(-translate.power, -translate.power);

		if (abs(translate.error) < 6  && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1 && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftTop.getActualVelocity() < 1 && leftBottom.getActualVelocity() < 1)
		{
			break;
		}

    timer += 10;
		pros::delay(10);
	}

	drive -> getModel() -> tank(0, 0);
}

void update() {

  QLength distL = 2_in * ((drive -> getModel() -> getSensorVals()[0]).get() - lPrev) / 180.0 * PI; // encoders measure things in degrees, so first convert to radians to get the distance that wheel has turned
  QLength distR = 2_in * (((drive -> getModel() -> getSensorVals()[1]) * 3 / 7).get() - rPrev) / 180.0 * PI;
  // QLength distM = 2_in * (midEnc.get() - mPrev) / 180.0 * PI;
  QLength forward = (distL + distR) / 2;
  QAngle dtheta = (distR - distL) / 13.7_in * radian; // small angle
  pos += forward * heading;
  curAngle += dtheta; // change angle
		// curAngle = fmod(curAngle.convert(radian) + 2 * PI, 2 * PI) * radian;
	heading = QVector<Number>(curAngle); // heading also changes
  // QLength side = distM - CHASSIS_RATIO * (distR - distL) / 2; // CHASSIS_RATIO is the ratio of the width of chassis to the distance of the middle wheel from the center of the bot
  lPrev = (drive -> getModel() -> getSensorVals()[0]); //removed the .get()
	rPrev = ((drive -> getModel() -> getSensorVals()[1]) * 3 / 7); //removed the .get()

}

// void moveOdom() {
//
// }
