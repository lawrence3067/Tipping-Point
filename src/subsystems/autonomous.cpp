#include "main.h"

using namespace okapi;

void rightAuton()
{
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(65, 2000);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  translatePID(-40, 2000);
  rotatePID(90, 1000);
  rotatePID(48, 700);
  autonTwoBarPID(-340);
  translatePID(-65, 2500);
  autonTwoBarPID(-100);
  translatePID(70, 4000);
}

void leftAuton()
{
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(65, 2100);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  translatePID(-40, 2000);
  rotatePID(105, 1500);
  rotatePID(104, 1500);
  autonTwoBarPID(-340);
  translatePID(-60, 2000);
  autonTwoBarPID(-100);
  translatePID(60, 2000);
}

void rightAutonWP()
{
  /**
  pros::c::adi_digital_write(pneumaticPort, LOW);
  autonTwoBarPID(-340);
  translatePID(-68, 2000);
  autonTwoBarPID(-50);
  translatePID(35, 1000);
  rotatePID(-100, 1000);
  translatePID(25, 700);
  autonFourBarPID(-100);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  autonFourBarPID(0);
  translatePID(10, 500);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  translatePID(-40, 2000);**/

  pros::c::adi_digital_write(pneumaticPort, HIGH);
  autonFourBarPID(-400);
  translatePID(20, 700);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  autonFourBarPID(0);
  translatePID(10, 500);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  autonFourBarPID(-800);
  rotatePID(45, 1000);
  rotatePID(45, 1000);
  autonFourBarPID(-10);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  autonTwoBarPID(-340);
  translatePID(-70, 2000);
  autonTwoBarPID(-50);
  translatePID(70, 3000);

}
