#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBack(rightBackPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBack(leftBackPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
IMU inertial_sensor(imuPort, IMUAxes::z);

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftBack}, {rightFront, rightBack})   //MotorGroups for left and right side
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 10_in}, imev5BlueTPR})		  //Blue gearset(100 rpm) and wheel dimensions
  .build();

typedef struct PID pid;

pid translate;
pid rotate;

double inertial_values;

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
	translate.kI = 0.0010;
	translate.kD = 0.00015;

	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

	drive -> getModel() -> resetSensors();

	while (true)
	{
		translate.error = translate.target - (drive -> getModel() -> getSensorVals()[0] + drive -> getModel() -> getSensorVals()[1])/2;
		translate.power = translateController.step(translate.error);

		drive -> getModel() -> tank(-translate.power, -translate.power);
    //pros::lcd::set_text(2, std::to_string(translate.error));

		if (abs(translate.error) < 10)
		{
			break;
		}
		pros::delay(10);
	}
	drive -> getModel() -> tank(0, 0);
}

void rotate_PID(double turn_degrees)
{
  drive -> getModel() -> setEncoderUnits(AbstractMotor::encoderUnits::degrees);

  //PID constants
  rotate.kP = 0.010150;
  rotate.kI = 0.0090;
  rotate.kD = 0.000005;

  auto rotateController = IterativeControllerFactory::posPID(rotate.kP, rotate.kI, rotate.kD);


  inertial_sensor.reset();

  while (true)
  {
    inertial_values = inertial_sensor.get();
    rotate.error = turn_degrees - inertial_values;
    rotate.speed = rotateController.step(rotate.error);                             //returns speed for left side

    //pros::lcd::set_text(2, std::to_string(rotate.error));
    drive -> getModel() -> tank(-rotate.speed, rotate.speed);

    if (abs(rotate.error) < 5)
    {
        break;
    }
    pros::delay(10);
  }
  drive -> getModel() -> tank(0, 0);                                 //brakes drivetrain right after PId movement
}
