#include "main.h"

double difference;

void rightAutonWP()
{
  fourBarClamp.set_value(false);
  tilter.set_value(false);
  tilterClamp.set_value(true);
  odom::driveToPoint({0_in, 48_in}, false, 1000, 500);
  odom::driveToPoint({0_in, 16.5_in}, true, 1000, 500);
  fourBarClamp.set_value(false);
  fourBarLift.moveVelocity(fourBarMacros(0));
  odom::driveToPoint({0_in, 10_in}, true, 500, 500);
  odom::driveToPoint({-34_in, 48_in}, false, 2000, 500);
  odom::driveToPoint({-15_in, 18_in}, true, 1000, 200);
  fourBarClamp.set_value(false);
  odom::driveToPoint({0_in, 13_in}, true, 1000, 1000);
  pros::delay(500);
  odom::driveToPoint({14_in, 17_in}, true, 1000, 1000);
  tilterClamp.set_value(false);
  pros::delay(750);
  tilter.set_value(true);
  conveyor.moveVelocity(-200);
  odom::driveToPoint({-3_in, 13_in}, false, 1000, 1000);

}

void leftAutonWP()
{

}

void skills()
{


}
/**
void test()
{
  inertialSensor.reset();
  odom::driveToPoint({24_in, 0_in}, false, 1000);
  odom::driveToPoint({24_in, 24_in}, true, 1000);
  odom::driveToPoint({0_in, 24_in}, false, 1000);
  odom::driveToPoint({0_in, 0_in}, true, 1000);
}**/
