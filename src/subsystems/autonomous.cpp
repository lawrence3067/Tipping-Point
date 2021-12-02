#include "main.h"

using namespace okapi;

void skillsHard()
{
  twoBarPID(-340);
  translatePID(-7, 500);
  twoBarPID(-50);
  rotatePID(90, 1500);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(55, 1500);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  rotatePID(45, 700);/**
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

void skillsPush()
{
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(120, 9000);

  translatePID(-10, 1000);
  rotatePID(35, 1000);
  twoBarPID(-340);

  translatePID(-65, 4000);
  pros::delay(5);
  translatePID(-25, 2500);
  //twoBarPID(-340);
  //translatePID(20);

  // translatePID(-90);
  // translatePID(20); //20

  pros::delay(5);
  rotatePID(-85, 2000);
  translatePID(70, 7000);
  /**
  jankRotatePID(80, 0);
  translatePID(-70);
  translatePID(20);
  jankRotatePID(0,50);
  translatePID(55);
  pros::delay(500);
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::delay(500);
  twoBarPID(10);
  jankRotatePID(40, 0);
  translatePID(-100);
  jankRotatePID(-90, 0);**/
}

void skillsPlatform()
{
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  translatePID(98, 3000);
  fourBarPID(-50);
  pros::c::adi_digital_write(pneumaticPort, LOW); //clamps blue
  fourBarPID(-800);
  rotatePID(86, 1500);
  translatePID(-15, 700);
  fourBarPID(0);
  translatePID(-57, 2500); //pushes rings out the way
  fourBarPID(-800);
  rotatePID(-90, 1500);
  translatePID(18, 700);
  fourBarPID(-600);
  pros::delay(500);
  pros::c::adi_digital_write(pneumaticPort, HIGH);
  fourBarPID(-800);
  translatePID(-10, 700);
  fourBarPID(0);
  translatePID(-75, 3000); //pushes large netural
  rotatePID(-75, 2000);
  translatePID(-100, 2000);
  translatePID(35, 1500);
  rotatePID(82, 1500);
  translatePID(-100, 2000);
  translatePID(90, 3000);
  pros::c::adi_digital_write(pneumaticPort, LOW); //clamps small neutral
  pros::delay(500);
  rotatePID(-45, 1000);
  fourBarPID(-800);
  translatePID(50, 1500);
  pros::delay(500);
  fourBarPID(-700);
  pros::c::adi_digital_write(pneumaticPort, HIGH); //put small neutral on platform
  translatePID(-10, 1000);
  rotatePID(110, 2000);
  fourBarPID(0);
  twoBarPID(-340);
  translatePID(-100, 4000);
  twoBarPID(-100);
  translatePID(17, 1000);
  rotatePID(90, 1500);
  translatePID(200, 10000);

}


void test()
{
  pros::c::adi_digital_write(pneumaticPort, LOW);
  pros::lcd::set_text(4, "fuck you");
}
