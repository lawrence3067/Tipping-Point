#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightTop(rightTopPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBottom(rightBottomPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftFront(leftFrontPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftTop(leftTopPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBottom(leftBottomPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

IMU inertialSensor(inertialPort, IMUAxes::z);
IMU inertialSensor2(inertial2Port, IMUAxes::y);

typedef struct PID pid;
pid park;
pid rotate1;
pid translate1;

int timer1;

std::shared_ptr<ChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftTop, leftBottom}, {rightFront, rightTop, rightBottom})
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 13.7_in}, imev5BlueTPR})
  .build();

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
void translatePID(double targetDistance, int ms, bool clamp)
{
  translate1.target = targetDistance * (360 / (2 * 3.1415 * (4 / 2)));

  timer1 = 0;

  translate1.kP = 0.001;
  translate1.kI = 0;
  translate1.kD = 0.000025;

  drive -> getModel() -> resetSensors();

  auto translate1Controller = IterativeControllerFactory::posPID(translate1.kP, translate1.kI, translate1.kD);

  while (timer1 < ms)
  {
    translate1.error = translate1.target - (drive -> getModel() -> getSensorVals()[0] * 18 / 35);
    translate1.power = translate1Controller.step(translate1.error);

    drive -> getModel() -> tank(-translate1.power, -translate1.power);

    if (abs(translate1.error) < 6  && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
      && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1
      && leftTop.getActualVelocity() < 1 && leftBottom.getActualVelocity() < 1)
    {
      drive -> getModel() -> tank(0, 0);
      break;
    }
    if (clamp == true)
    {
      if (fourBarSwitch.get_new_press() == 1 || fourBarSwitch2.get_new_press() == 1)
      {
        fourBarClamp.set_value(false);
        drive -> getModel() -> tank(0, 0);
        break;
      }
    }

    timer1 += 10;
    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);
}

void rotatePID(double targetAngle, int ms)
{
  inertialSensor.reset();
  timer1 = 0;

  rotate1.kP = 0.0328;
  rotate1.kI = 0.004;
  rotate1.kD = 0.0008;

  auto rotate1Controller = IterativeControllerFactory::posPID(rotate1.kP, rotate1.kI, rotate1.kD);

  while (timer1 < ms)
  {
    rotate1.error = targetAngle - inertialSensor.get();
    rotate1.power = rotate1Controller.step(rotate1.error);

    drive -> getModel() -> tank(-rotate1.power, rotate1.power);

    if (abs(rotate1.error) < 2 && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
      && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftTop.getActualVelocity() < 1
      && leftBottom.getActualVelocity() < 1)
    {
      drive -> getModel() -> tank(0, 0);
      break;
    }

    timer1 += 10;
    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);
}

void translateJank(double targetDistance, int ms, bool clamp)
{
  translate1.target = targetDistance * (360 / (2 * 3.1415 * (4 / 2)));

  timer1 = 0;

  translate1.kP = 0.001;
  translate1.kI = 0;
  translate1.kD = 0.000025;

  drive -> getModel() -> resetSensors();

  auto translate1Controller = IterativeControllerFactory::posPID(translate1.kP, translate1.kI, translate1.kD);
  translate1Controller.setOutputLimits(-0.2, 0.2);

  while (timer1 < ms)
  {
    translate1.error = translate1.target - (drive -> getModel() -> getSensorVals()[0] * 18 / 35);
    translate1.power = translate1Controller.step(translate1.error);

    drive -> getModel() -> tank(-translate1.power, -translate1.power);

    if (abs(translate1.error) < 6  && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
      && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1
      && leftTop.getActualVelocity() < 1 && leftBottom.getActualVelocity() < 1)
    {
      drive -> getModel() -> tank(0, 0);
      break;
    }
    if (clamp == true)
    {
      if (fourBarSwitch.get_new_press() == 1 || fourBarSwitch2.get_new_press() == 1)
      {
        fourBarClamp.set_value(false);
        drive -> getModel() -> tank(0, 0);
        break;
      }
    }

    timer1 += 10;
    pros::delay(10);
  }

  drive -> getModel() -> tank(0, 0);
}
