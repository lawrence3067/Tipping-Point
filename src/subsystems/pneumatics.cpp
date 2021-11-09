#include "main.h"

void updatePneumatics()
{
   if (controller.getDigital(ControllerDigital::up) == 1)
   {
     pros::c::adi_digital_write(pneumaticPort, HIGH);
   }
   else if (controller.getDigital(ControllerDigital::down) == 1)
   {
     pros::c::adi_digital_write(pneumaticPort, LOW);
   }
}
