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
/**
void autonPark()
{
  inertialSensor2.reset();

  leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

  rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

  park.kP = 0.01;
  park.kI = 0;
  park.kD = 0.0005;

  auto parkController = IterativeControllerFactory::posPID(park.kP, park.kI, park.kD);

  while (abs(inertialSensor2.get()) > 5)
  {
    park.error = inertialSensor2.get();
    park.power = parkController.step(park.error);

    drive -> getModel() -> tank(park.power, park.power);

    pros::delay(10);
  }

  drive -> getModel() -> tank(0,0);
}**/
