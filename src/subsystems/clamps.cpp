#include "main.h"

void updateFourBarClamp()
{
  if (controller.getDigital(ControllerDigital::up) == 1)
  {
    pros::c::adi_digital_write(fourBarClampPort, HIGH);
  }
  else if (controller.getDigital(ControllerDigital::down) == 1)
  {
    pros::c::adi_digital_write(fourBarClampPort, LOW);
  }
}

void updateTilterClamp()
{
  if (controller.getDigital(ControllerDigital::left) == 1)
  {
    pros::c::adi_digital_write(tilterClampPort, HIGH);
  }
  else if (controller.getDigital(ControllerDigital::right) == 1)
  {
    pros::c::adi_digital_write(tilterClampPort, LOW);
  }
}
