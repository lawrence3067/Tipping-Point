#include "main.h"

using namespace okapi;

Motor rightFront(rightFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightTop(rightTopPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor rightBottom(rightBottomPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor leftFront(leftFrontPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftTop(leftTopPort, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor leftBottom(leftBottomPort, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

ADIEncoder leftEncoder(leftEncoder1, leftEncoder2, false);
ADIEncoder rightEncoder(rightEncoder1, rightEncoder2, true);

std::shared_ptr<OdomChassisController> drive =
  ChassisControllerBuilder()
  .withMotors({leftFront, leftTop, leftBottom}, {rightFront, rightTop, rightBottom})
  .withDimensions(AbstractMotor::gearset::blue, {{4_in, 13.5_in}, imev5BlueTPR})
  .withSensors(leftEncoder, rightEncoder)
  .withOdometry({{3.25_in, 2.8_in}, quadEncoderTPR}, StateMode::CARTESIAN)
  .withGains(
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0}
  )
  .buildOdometry();

void updateDrive()
{
  drive -> getModel() -> tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}
