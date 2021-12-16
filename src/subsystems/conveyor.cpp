#include "main.h"

using namespace okapi;

Motor conveyor(conveyorPort, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

int conveyorState;

void updateConveyor()
{
  if (controller.getDigital(ControllerDigital::R1) == 1)
  {
    conveyorState = 1;
  }
  else if (controller.getDigital(ControllerDigital::R2) == 1)
  {
    conveyorState = 2;
  }

  switch (conveyorState)
  {
    case 1:
      conveyor.moveVelocity(200);
      break;
    case 2:
      conveyor.moveVelocity(-200);
      break;
  }
}
