#include "main.h"

using namespace okapi;

Motor twoBarLift(twoBarLiftPort, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

int twoBarButtonCount;

typedef struct PID pid;

pid TB;

double twoBarPID(double setpoint)
{
  TB.kP = 0.7;
  TB.kI = 0;
  TB.kD = 0.05;

  TB.target = setpoint;
  TB.error = TB.target - twoBarLift.getPosition();
  TB.integral += TB.error;
  TB.derivative = TB.error - TB.prevError;
  TB.power = TB.kP * TB.error + TB.kI * TB.integral + TB.kD * TB.derivative;

  return TB.power;
}

void updateTwoBarLiftMacro()
{
  if (controller.getDigital(ControllerDigital::R1) == 1)
  {
    twoBarButtonCount = 2;
  }
  else if(controller.getDigital(ControllerDigital::R2) == 1)
  {
    twoBarButtonCount = 1;
  }

  switch (twoBarButtonCount)
  {
  case 1:
    twoBarLift.moveVelocity(twoBarPID(-340));
    break;
  case 2:
    twoBarLift.moveVelocity(twoBarPID(-100));
    break;
  }

}
