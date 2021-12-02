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
  int brakeMode;

void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));

  if (controller.getDigital(ControllerDigital::Y) == 1)
  {
    rightFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightBack.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftFront.setBrakeMode(AbstractMotor::brakeMode::coast);
    leftBack.setBrakeMode(AbstractMotor::brakeMode::coast);
  }

}


void translatePID(double distance, int ms)
{
  timer = 0;
  inertial_sensor.reset();
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	translate.target = distance * (360 / (2 * 3.1415 * (4 / 2)));

	//PID constants
	translate.kP = 0.0045;
	translate.kI = 0;
	translate.kD = 0.0002;

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

	drive -> getModel() -> resetSensors();

	while (timer < ms)
	{

    inertial_value = inertial_sensor.get();
		translate.error = translate.target - ((drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2 * 18 / 35);
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
    error = target - inertial_sensor.get();
    integral = integral + error;
    derivative = error - prevError;
    prevError = error;
    p = error * kP;
    i = error * ki;
    d = error * kd;

    vel = p + i + d;

    drive -> getModel() -> tank(vel, -vel);

    if(abs(error) < threshold && rightFront.getActualVelocity() < 1 && rightBack.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftBack.getActualVelocity() < 1)
    {
      break;
    }

    timer += 10;
    pros::delay(10);

  }
}
