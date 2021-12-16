#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

int liftState;
double fourBarPower;
double error;
auto fourBarController = IterativeControllerFactory::posPID(0.85, 0, 0.05);;

void fourBarMacros(double setpoint)
{
  error = setpoint - fourBarLift.getPosition();
  fourBarPower = fourBarController.step(error);

  if (fourBarController.isSettled() == true)
  {
    fourBarLift.moveVelocity(10);
  }
  else
  {
    fourBarLift.moveVelocity(fourBarPower);
  }
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

  switch (liftState)
  {
    case 1:
      fourBarLift.moveVelocity(100);
      break;
    case 2:
      fourBarLift.moveVelocity(-100);
      break;
  }
}
