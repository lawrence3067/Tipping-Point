#include "main.h"

using namespace okapi;

Motor twoBarLift(twoBarLiftPort, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

typedef struct PID pid;

pid TB;

void twoBarPID(double setpoint)
{
  while (true)
  {
    TB.kP = 0.5;
    TB.kI = 0.001;
    TB.kD = 0.05;

    TB.target = setpoint;
    TB.error = TB.target - twoBarLift.getPosition();
    TB.integral += TB.error;
    TB.derivative = TB.error - TB.prevError;
    TB.power = TB.kP * TB.error + TB.kI * TB.integral + TB.kD * TB.derivative;

    twoBarLift.moveVelocity(TB.power);

    if (abs(TB.error) < 7)
    {
      twoBarLift.setBrakeMode(AbstractMotor::brakeMode::hold);
      break;
    }

    pros::delay(10);
  }
}
