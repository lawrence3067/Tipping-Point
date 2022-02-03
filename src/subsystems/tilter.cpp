#include "main.h"

pros::ADIDigitalOut tilter(tilterPort);
pros::ADIDigitalOut tilterClamp(tilterClampPort);

pros::ADIDigitalIn tilterSwitch(tilterSwitchPort);

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::B) == 1)
  {
    tilter.set_value(false);
    pros::delay(250);
    tilterClamp.set_value(true);
  }
  else if (controller.getDigital(ControllerDigital::X) == 1 || tilterSwitch.get_new_press() == 1)
  {
    tilterClamp.set_value(false);
    pros::delay(500);
    tilter.set_value(true);
  }
}
