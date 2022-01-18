#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarPort, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

int liftState;
double fourBarPower;
double error;
auto fourBarController = IterativeControllerFactory::posPID(1.5, 0, 0.001);

double fourBarMacros(double setpoint)
{
  fourBarController.setOutputLimits(-100, 100);
  error = setpoint - fourBarLift.getPosition();
  fourBarPower = fourBarController.step(error);

  return (-fourBarPower);
}

void updateFourBar()
{
  if (controller.getDigital(ControllerDigital::L1) == 1)
  {
    liftState = 1;
  }
  else if (controller.getDigital(ControllerDigital::L2) == 1)
  {
    liftState = 2;
  }
  else if (controller.getDigital(ControllerDigital::A) == 1)
  {
    liftState = 3;
  }
  else if (controller.getDigital(ControllerDigital::Y) == 1)
  {
    liftState = 4;
  }

  switch (liftState)
  {
    case 1:
      fourBarLift.moveVelocity(fourBarMacros(700));
      break;
    case 2:
      fourBarLift.moveVelocity(fourBarMacros(-20));
      break;
    case 3:
      fourBarLift.moveVelocity(fourBarMacros(500));
      break;
    case 4:
      fourBarLift.moveVelocity(fourBarMacros(550));
  }
}
