#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
IMU inertial_sensor(IMUPort, IMUAxes::z);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})
  .build();

typedef struct PID pid;

pid translate;
pid rotate;
pid jankLeft;
pid jankRight;

double inertial_values;
double error;
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
double inertial_value;

void translatePID(double distance)
{
  inertial_sensor.reset();
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	translate.target = distance * (360 / (2 * 3.1415 * (4 / 2)));

	//PID constants
	translate.kP = 0.000575; //.002075
	translate.kI = 0.05; //0.05
	translate.kD = 0.0002;

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

	drive -> getModel() -> resetSensors();

	while (true)
	{

    inertial_value = inertial_sensor.get();
		translate.error = translate.target - ((drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2 * 18 / 35);
		translate.power = translateController.step(translate.error);

    if (inertial_value > 0 && distance > 0)
    {
      drive -> getModel() -> tank(-translate.power, abs(translate.power) + inertial_value * 0.012);
    }
    else if (inertial_value < 0 && distance > 0)
    {
      drive -> getModel() -> tank(abs(translate.power) + inertial_value * 0.012, -translate.power);
    }
    else
    {
      drive -> getModel() -> tank(-translate.power, -translate.power);
    }

		if (abs(translate.error) < 12)
		{
			break;
		}

		pros::delay(10);
	}

	drive -> getModel() -> tank(0, 0);
}

void rotatePID(double target)
{
  inertial_sensor.reset();
  error = target - inertial_sensor.get();
  threshold = 2; //2
  kP = 1.6; //0.8
  ki = 0.02;
  kd = 0.03; //3.5

  while (true)
  {
    error = target - inertial_sensor.get();
    integral = integral + error;
    derivative = error - prevError;
    prevError = error;
    p = error * kP;
    i = error * ki;
    d = error * kd;

    vel = p + i + d;

    rightFront.moveVelocity(-vel);
    leftFront.moveVelocity(vel);
    rightBack.moveVelocity(-vel);
    leftBack.moveVelocity(vel);

    if(abs(error) < threshold)
    {
      break;
    }

    pros::delay(10);
  }
  drive -> getModel() -> tank(0, 0);

}

void jankRotatePID(double leftDistance, double rightDistance)
{
  jankLeft.target = leftDistance * 360 / (4 * M_PI);
  jankRight.target - rightDistance * 360 / (4 * M_PI);

  jankLeft.kP = 0.000775;
  jankLeft.kI = 0.05;
  jankLeft.kD = 0.0004;

  auto jankLeftController = IterativeControllerFactory::posPID(jankLeft.kP, jankLeft.kI, jankLeft.kD);
  auto jankRightController = IterativeControllerFactory::posPID(jankLeft.kP, jankLeft.kI, jankLeft.kD);

  drive -> getModel() -> resetSensors();

  while (true)
  {
    jankLeft.error = jankLeft.target - drive -> getModel() -> getSensorVals()[0];
    jankRight.error = jankRight.target - drive -> getModel() -> getSensorVals()[1];

    jankLeft.power = jankLeftController.step(jankLeft.error);
    jankRight.power = jankRightController.step(jankRight.error);

    drive -> getModel() -> tank(-jankLeft.power, -jankRight.power);

    if (abs(jankLeft.error) < 1 && abs(jankRight.error) < 1)
    {
      break;
    }
    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);

}
