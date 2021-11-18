#include "main.h"

using namespace okapi;

void skillsHard()
{
  twoBarPID(-340);
  translatePID(-9);
  twoBarPID(10);
  //translatePID(8, 8);
  pros::delay(500);
  rotatePID(90);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(105);
  pros::delay(500);
  translatePID(-10);
  rotatePID(-90);
  twoBarPID(-340);
  translatePID(12);
  pros::delay(500);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  rotatePID(30);
  translatePID(-75); //130
  rotatePID(45);
  translatePID(-30);
  translatePID(10);
  rotatePID(-30);
  translatePID(-50);
  twoBarPID(10);
  /**
  translatePID(8, 8);
  rotatePID(-30);
  translatePID(-8, -8);

**/
  // rotatePID(30);
  // translatePID(62, 62);
  // fourBarPID(-800);
  // rotatePID(-16);
  // translatePID(7, 7);
  // fourBarPID(-600);
  // pros::c::adi_digital_write(pneumaticPort, HIGH);



  /**
  rotatePID(55);
  twoBarPID(-340);
  translatePID(36, 36);
  rotatePID(90);
  translatePID(-12, -12);
**/
}

void skillsBackup()
{

}
