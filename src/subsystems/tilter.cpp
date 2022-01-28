#include "main.h"

pros::ADIDigitalOut tilter(tilterPort);
pros::ADIDigitalOut tilterClamp(tilterClampPort);

pros::ADIDigitalIn tilterSwitch(tilterSwitchPort);

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::B) == 1)
  {
    /**
    pros::c::adi_digital_write(tilterPort, LOW);
    pros::delay(250);
    pros::c::adi_digital_write(tilterClampPort, HIGH);**/
    tilter.set_value(false);
    pros::delay(250);
    tilterClamp.set_value(true);
  }
  else if (controller.getDigital(ControllerDigital::X) == 1 || tilterSwitch.get_value() == 1)
  {/**
    pros::c::adi_digital_write(tilterClampPort, LOW);
    pros::delay(500);
    pros::c::adi_digital_write(tilterPort, HIGH);**/
    tilterClamp.set_value(false);
    pros::delay(500);
    tilter.set_value(true);
  }
}
