#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})
  .build();

typedef struct PID pid;

pid translate;
pid rotate;

void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void translatePID(double leftDistance, double rightDistance)
{
	drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

	translate.target = leftDistance * (360 / (2 * 3.1415 * (4 / 2)));

	//PID constants
	translate.kP = 0.001575;
	translate.kI = 0.0015;
	translate.kD = 0.00015;

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

	drive -> getModel() -> resetSensors();

	while (true)
	{
		translate.error = translate.target - ((drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2 * 18 / 35);
		translate.power = translateController.step(translate.error);

		drive -> getModel() -> tank(-translate.power, -translate.power);

    pros::lcd::set_text(2, std::to_string(translate.error));

		if (abs(translate.error) < 10)
		{
			break;
		}

		pros::delay(10);
	}
	drive -> getModel() -> tank(0, 0);
}

void rotatePID(double turnDegrees)
{
  double wheelLeft;
  double wheelRight;

  double leftVals;
  double rightVals;

  double deltaLeft;
  double deltaRight;

  double prevLeftVals;
  double prevRightVals;

  double newTheta;

  rotate.kP = 0.001575;   //0.001575
	rotate.kI = 0.0015; //0.0015
	rotate.kD = 0.00015; //0.00015

  drive -> getModel() -> resetSensors();

  while (true)
  {
    leftVals = (drive -> getModel() -> getSensorVals()[0]) * 18 / 35;   //corrects gear ratio and shit motor encoders
    rightVals = (drive -> getModel() -> getSensorVals()[1]) * 18 / 35;

    leftVals = leftVals * 4 * M_PI / 360;     //converts degrees to inches
    rightVals = rightVals * 4 * M_PI / 360;

    newTheta = (leftVals - rightVals) / (6 + 6); //calculates angle change
    newTheta = newTheta / M_PI * 180;   //converts from radians to degrees

    rotate.target = turnDegrees;
    rotate.error = rotate.target - newTheta;
    pros::lcd::set_text(3, std::to_string(rotate.error));
    rotate.integral += rotate.error;
    rotate.derivative = rotate.error - rotate.prevError;
    rotate.power = (rotate.kP * rotate.error) + (rotate.kI * rotate.integral) + (rotate.kD * rotate.derivative);

    if (abs(rotate.error) < 1)
    {
      break;
    }

    drive -> getModel() -> tank(rotate.power, -rotate.power);

    pros::delay(10);

  }

  drive -> getModel() -> tank(0, 0);
}
