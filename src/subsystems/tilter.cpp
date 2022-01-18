#include "main.h"

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::X) == 1)
  {
    pros::c::adi_digital_write(tilterClampPort, HIGH);
    pros::delay(250);
    pros::c::adi_digital_write(tilterPort, LOW);
  }
  else if (controller.getDigital(ControllerDigital::B) == 1)
  {
    pros::c::adi_digital_write(tilterPort, HIGH);
    pros::c::adi_digital_write(tilterClampPort, LOW);
  }
}
