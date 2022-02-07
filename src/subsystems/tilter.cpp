#include "main.h"

pros::ADIDigitalOut tilter(tilterPort);
pros::ADIDigitalOut tilterClamp(tilterClampPort);

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::B) == 1)
  {
    tilter.set_value(false);
    pros::delay(250);
    tilterClamp.set_value(true);
  }
  else if (controller.getDigital(ControllerDigital::X) == 1)
  {
    tilterClamp.set_value(false);
    pros::delay(750);
    tilter.set_value(true);
  }
}
