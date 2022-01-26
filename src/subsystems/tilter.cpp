#include "main.h"

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::B) == 1)
  {
    pros::c::adi_digital_write(tilterPort, LOW);
    pros::delay(500);
    pros::c::adi_digital_write(tilterClampPort, HIGH);
  }
  else if (controller.getDigital(ControllerDigital::X) == 1)
  {
    pros::c::adi_digital_write(tilterClampPort, LOW);
    pros::delay(500);
    pros::c::adi_digital_write(tilterPort, HIGH);
  }
}
