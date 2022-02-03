#include "main.h"

double difference;

void rightAutonWP()
{
  inertialSensor2.reset();
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  pros::c::adi_digital_write(tilterPort, HIGH);
  pros::c::adi_digital_write(tilterClampPort, LOW);
  translatePID(62, 1500);
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  translatePID(-30, 1000);
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  translatePID(-15, 1000);
  difference = -40 - inertialSensor2.get();
  rotatePID(difference, 500);
  translatePID(54, 1200);
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  pros::delay(500);
  translatePID(-41, 1200);
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  translatePID(-10, 500);
  difference = -95 - inertialSensor2.get();
  rotatePID(difference, 800);
  fourBarLift.moveVelocity(fourBarMacros(700));
  translatePID(-27, 1000);
  pros::c::adi_digital_write(tilterClampPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(tilterPort, LOW);
  pros::delay(1000);
  conveyor.moveVelocity(-200);
  translatePID(30, 2000);


}

void leftAutonWP()
{
  inertialSensor2.reset();
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  pros::c::adi_digital_write(tilterPort, HIGH);
  pros::c::adi_digital_write(tilterClampPort, LOW);
  translatePID(55, 1700);
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  pros::delay(250);
  translatePID(-28, 1500);
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  fourBarLift.moveVelocity(fourBarMacros(300));
  translatePID(-13, 1000);
  difference = -48 - inertialSensor2.get();
  rotatePID(difference, 2000); //pick up red alliance
  translatePID(-19, 700);
  pros::c::adi_digital_write(tilterClampPort, HIGH);
  pros::delay(200);
  translatePID(15, 700);
  fourBarLift.moveVelocity(fourBarMacros(700));
  pros::delay(1000);
  conveyor.moveVelocity(-200);
  pros::delay(1000);
  pros::c::adi_digital_write(tilterClampPort, LOW);
  difference = 30 - inertialSensor2.get();
  rotatePID(difference, 1000);
  translatePID(80, 1700);
  fourBarLift.moveVelocity(fourBarMacros(0));
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  translatePID(-80, 1700);

}

void skills(){
  inertialSensor2.reset();
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  pros::c::adi_digital_write(tilterPort, HIGH);
  pros::c::adi_digital_write(tilterClampPort, LOW);
  translatePID(62, 1500);
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  translatePID(-30, 1000);
  pros::c::adi_digital_write(fourBarClampPort, LOW);
  translatePID(-15, 1000);
  difference = -40 - inertialSensor2.get();
  rotatePID(difference, 500);
  translatePID(70, 1200);
  translatePID(-57, 1200);
  translatePID(-10, 500);
  difference = -95 - inertialSensor2.get();
  rotatePID(difference, 800);
  fourBarLift.moveVelocity(fourBarMacros(700));
  translatePID(-27, 1000);
  pros::c::adi_digital_write(tilterClampPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(tilterPort, LOW);
  pros::delay(1000);
  conveyor.moveVelocity(-200);
  translatePID(30, 2000);

  conveyor.moveVelocity(0);
  fourBarLift.moveVelocity(fourBarMacros(0));
  difference = -90-inertialSensor2.get();
  rotatePID(difference,800);
  translatePID(84, 2000);
  difference = 0 - inertialSensor2.get();
  rotatePID(difference,800);
  translatePID(100,2000);

  difference = -90 - inertialSensor2.get();
  translatePID(-10,2000);
  rotatePID(difference ,2000);
  translatePID(20, 500);
  //unclamp tilter?
  pros::c::adi_digital_write(fourBarClampPort, HIGH);
  difference = -180 - inertialSensor2.get();
  rotatePID(difference, 1000);

  translatePID(100,2000);
  pros::c::adi_digital_write(fourBarClampPort, LOW);

}

void test()
{
  inertialSensor.reset();
  odom::driveToPoint({24_in, 0_in}, false, 1000);
  odom::driveToPoint({24_in, 24_in}, false, 1000);
  odom::driveToPoint({0_in, 24_in}, false, 1000);
  odom::driveToPoint({0_in, 0_in}, false, 1000);
}
