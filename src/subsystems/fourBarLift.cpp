#include "main.h"

using namespace okapi;

Motor fourBarLift(fourBarLiftPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

typedef struct PID pid;

pid FB;

void fourBarPID(double setpoint)
{
  while (true)
  {
    FB.kP = 0.55;
    FB.kI = 0;
    FB.kD = 0.05;

    FB.target = setpoint;
    FB.error = FB.target - fourBarLift.getPosition();
    FB.integral += FB.error;
    FB.derivative = FB.error - FB.prevError;
    FB.power = FB.kP * FB.error + FB.kI * FB.integral + FB.kD * FB.derivative;

    fourBarLift.moveVelocity(FB.power);

    if (abs(FB.error) < 7)
    {
      fourBarLift.setBrakeMode(AbstractMotor::brakeMode::brake);
      break;
    }

    pros::delay(10);
  }
}
