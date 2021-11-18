#include "main.h"

void updatePneumatics(bool state)
{
   if (state == false)
   {
     pros::c::adi_digital_write(pneumaticPort, HIGH);
   }
   else if (state == true)
   {
     pros::c::adi_digital_write(pneumaticPort, LOW);
   }
}
