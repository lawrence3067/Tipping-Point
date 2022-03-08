#include "main.h"

pros::ADIDigitalOut fourBarClamp(fourBarClampPort);
pros::ADIDigitalIn fourBarSwitch(fourBarSwitchPort);
pros::ADIDigitalIn fourBarSwitch2(fourBarSwitchPort2);

void updateFourBarClamp()
{
  if (controller.getDigital(ControllerDigital::up) == 1)
  {
    fourBarClamp.set_value(true);
  }
  else if (controller.getDigital(ControllerDigital::down) == 1  || fourBarSwitch.get_new_press() == 1 || fourBarSwitch2.get_new_press() == 1)
  {
    fourBarClamp.set_value(false);
  }
}
