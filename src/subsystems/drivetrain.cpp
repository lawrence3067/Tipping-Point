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
double derivative1;
double derivative2;
int timer;

void translatePID(double distance, int ms)
{
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	translate.target = distance * (360 / (2 * 3.1415 * (4 / 2)));

	//PID constants
	translate.kP = 0.0045;//0.003505   0.0055   //0.0037
	translate.kI = 0;//0.00001
	translate.kD = 0.0002;//0.00007

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

	drive -> getModel() -> resetSensors();

  timer = 0;

	while (timer < ms)
	{
		translate.error = translate.target - ((drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2 * 18 / 35);
    pros::lcd::set_text(3, std::to_string(translate.error));
		translate.power = translateController.step(translate.error);

    drive -> getModel() -> tank(-translate.power, -translate.power);

		if (abs(translate.error) < 6  && (rightFront.getActualVelocity() < 0.5 && rightBack.getActualVelocity() < 0.5 && leftFront.getActualVelocity() < 0.5 && leftBack.getActualVelocity() < 0.5))
		{
			break;
		}

    timer += 10;

		pros::delay(10);
	}

	drive -> getModel() -> tank(0, 0);
}

void rotatePID(double target, int ms)
{
  inertial_sensor.reset();
  timer = 0;
  error = target;
  threshold = 3;
  kP = 0.016;  //0.0158
  ki = 0.00003;
  kd = 0.0084;

  while (timer < ms)
  {
    pros::lcd::set_text(2, std::to_string(inertial_sensor.get()));
    error = target - inertial_sensor.get();
    integral = integral + error;
    derivative = error - prevError;
    prevError = error;
    p = error * kP;
    i = integral * ki;
    d = derivative * kd;

    vel = p + i + d;
    pros::lcd::set_text(5, std::to_string(vel));

    drive -> getModel() -> tank(vel, -vel);

    if(abs(error) < threshold && rightFront.getActualVelocity() < 1 && rightBack.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftBack.getActualVelocity() < 1)
    {
      break;
    }

    timer += 10;

    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);

}

void jankRotatePID(double leftDistance, double rightDistance)
{
  jankLeft.target = leftDistance * 360 / (360 / (2 * 3.1415 * (4 / 2)));
  jankRight.target = rightDistance * 360 / (360 / (2 * 3.1415 * (4 / 2)));

  jankLeft.kP = 0.006;
  jankLeft.kI = 0;
  jankLeft.kD = 0.0001;

  auto jankLeftController = IterativeControllerFactory::posPID(jankLeft.kP, jankLeft.kI, jankLeft.kD);
  auto jankRightController = IterativeControllerFactory::posPID(jankLeft.kP, jankLeft.kI, jankLeft.kD);

  drive -> getModel() -> resetSensors();

  while (true)
  {
    jankLeft.error = jankLeft.target - drive -> getModel() -> getSensorVals()[0];
    jankRight.error = jankRight.target - drive -> getModel() -> getSensorVals()[1];

    derivative1 = jankLeft.error - jankLeft.prevError;
    derivative2 = jankRight.error - jankRight.prevError;

    jankLeft.prevError = jankLeft.error;
    jankRight.prevError = jankRight.error;


    jankLeft.power = jankLeftController.step(jankLeft.error);
    jankRight.power = jankRightController.step(jankRight.error);

    drive -> getModel() -> tank(-jankLeft.power, -jankRight.power);

    if (abs(jankLeft.error) < 7 && abs(jankRight.error) < 7)
    {
      break;
    }
    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);

}
