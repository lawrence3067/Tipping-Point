#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

int liftButtonCount;

typedef struct PID pid;

pid FB;

double PIDFourBar(double setpoint)
{
  FB.kP = 0.5;
  FB.kI = 0;
  FB.kD = 0.05;

  FB.target = setpoint;
  FB.error = FB.target - fourBarLift.getPosition();
  FB.integral += FB.error;
  FB.derivative = FB.error - FB.prevError;

  FB.power = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;

  return FB.power;
}

void updateFourBarMacro()
{
  if (controller.getDigital(ControllerDigital::L1) == 1)
  {
    liftButtonCount = 3;
  }

  else if(controller.getDigital(ControllerDigital::L2) == 1)
  {
    liftButtonCount = 1;
    if(liftButtonCount > 1){
      liftButtonCount = 1;
    }
  }

  switch (liftButtonCount)
  {
  case 1:
    fourBarLift.moveVelocity(PIDFourBar(0));
    liftButtonCount += 1;
    break;
  case 2:
    fourBarLift.moveVelocity(PIDFourBar(250));
    break;
  case 3:
    fourBarLift.moveVelocity(PIDFourBar(800));
    break;
  }
  /**
  if (liftButtonCount == 3)
  {
    fourBarLift.moveVelocity(PIDFourBar(800));
  }
  else if(liftButtonCount == 2){
    fourBarLift.moveVelocity(PIDFourBar(250));
  }
  else if (liftButtonCount == 1)
  {
    fourBarLift.moveVelocity(PIDFourBar(0));
    liftButtonCount+=1;
  }
  **/
}
