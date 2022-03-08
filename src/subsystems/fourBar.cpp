#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarPort, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

int liftState;
double fourBarPower;
double error;
auto fourBarController = IterativeControllerFactory::posPID(0.5, 0, 0.001);//1.4, 0, 0.003
double setpoint;
bool manual;

double fourBarMacros(double setpoint)
{
  fourBarController.setOutputLimits(-100, 100);
  error = setpoint - fourBarLift.getPosition();
  fourBarPower = fourBarController.step(error);

  return (-fourBarPower);
}

void updateFourBar()
{
  fourBarLift.setBrakeMode(AbstractMotor::brakeMode::hold);
  setpoint = fourBarLift.getPosition();

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
      fourBarLift.moveVelocity(fourBarMacros(600));
      break;
    case 2:
      fourBarLift.moveVelocity(fourBarMacros(-10));
      break;
    case 3:
      fourBarLift.moveVelocity(fourBarMacros(250));
      break;
    case 4:
      fourBarLift.moveVelocity(fourBarMacros(550));
      break;
    case 5:
      fourBarLift.moveVelocity(fourBarMacros(setpoint));
      break;
    case 6:
      fourBarLift.moveVelocity(200);
      break;
    case 7:
      fourBarLift.moveVelocity(-200);
    case 8:
      fourBarLift.moveVelocity(0);
  }
/**
  if (controller.getDigital(ControllerDigital::L1) == 1 || controller.getDigital(ControllerDigital::L2) == 1)
  {
    manual = true;
  }
  else if (controller.getDigital(ControllerDigital::A) == 1)
  {
    manual = false;
  }

  if (manual == true)
  {
    fourBarLift.moveVelocity(200 * controller.getDigital(ControllerDigital::L1) - controller.getDigital(ControllerDigital::L2));
    setpoint = fourBarLift.getPosition();

    if (controller.getDigital(ControllerDigital::L1) == 0 && controller.getDigital(ControllerDigital::L2) == 0)
    {
      fourBarLift.moveVelocity(setpoint);
    }
  }
  else if (manual == false)
  {
    fourBarLift.moveVelocity(fourBarMacros(250));
  }**/
}
