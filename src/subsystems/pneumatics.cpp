#include "main.h"

void updatePneumatics()
{
   if (controller.getDigital(ControllerDigital::B) == 1)
   {
     pros::lcd::set_text(2, "YOUR MOM");
   }
   else
   {
     pros::c::adi_digital_write(pneumaticLeftPort, HIGH);
   }
}
