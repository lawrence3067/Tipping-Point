#include "main.h"

using namespace okapi;

void rightAuton()
{
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  twoBarLift.moveVelocity(twoBarPID(-340));
  translatePID(-65);
  twoBarLift.moveVelocity(twoBarPID(-100));
  pros::delay(1000);
  translatePID(40);
  jankRotatePID(-50, 50);
  twoBarLift.moveVelocity(twoBarPID(-340));
  pros::delay(500);
  jankFourBarPID(-400);
  translatePID(10);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
}

void leftAuton()
{
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(75);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(1000);
  translatePID(-55);
  jankRotatePID(-80,0);
  twoBarLift.moveVelocity(-340);
  translatePID(-10);
  twoBarLift.moveVelocity(0);
  translatePID(30);

}
