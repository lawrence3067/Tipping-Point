#include "main.h"

void updateTilter()
{
  if (controller.getDigital(ControllerDigital::X) == 1)
  {
    pros::c::adi_digital_write(rightTilterPort, HIGH);
    pros::c::adi_digital_write(leftTilterPort, HIGH);
  }
  else if (controller.getDigital(ControllerDigital::B) == 1)
  {
    pros::c::adi_digital_write(rightTilterPort, LOW);
    pros::c::adi_digital_write(leftTilterPort, LOW);
  }
}
