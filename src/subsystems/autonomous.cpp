#include "main.h"

void rightAutonWP()
{
  inertialSensor.reset();
  fourBarClamp.set_value(true);
  tilter.set_value(true);
  tilterClamp.set_value(false);
  odom::driveToPoint({0_in, 54_in}, false, 1500, 1);
  fourBarClamp.set_value(false);
  odom::driveToPoint({0_in, 15_in}, true, 20000, 500);
  /**
  fourBarLift.moveVelocity(fourBarMacros(0));
  fourBarClamp.set_value(true);
  odom::driveToPoint({0_in, 10_in}, true, 400, 500);
  pros::delay(500);
  odom::driveToPoint({-32_in, 48_in}, false, 2000, 500);
  fourBarClamp.set_value(false);
  odom::driveToPoint({-13_in, 17_in}, true, 1000, 200);
  fourBarClamp.set_value(true);
  odom::driveToPoint({0_in, 13_in}, true, 1000, 1000);
  fourBarLift.moveVelocity(fourBarMacros(0));
  odom::driveToPoint({0_in, 28_in}, false, 1000, 1000);
  fourBarClamp.set_value(false);**/
  pros::delay(500);
  odom::driveToPoint({14_in, 16_in}, true, 1000, 1000);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
  conveyor.moveVelocity(-200);
  odom::driveToPoint({-15_in, 13_in}, false, 1000, 1000);
}

void leftAutonWP()
{
  fourBarClamp.set_value(false);
  fourBarLift.moveVelocity(fourBarMacros(-10));
  odom::driveToPoint({0_in, 50_in}, false, 1000, 1);
  fourBarClamp.set_value(true);
  //odom::driveToPoint({0_in, 18_in}, true, 1000, 100);
  odom::driveToPoint({0_in, 3_in}, true, 1000, 100);
  //odom::driveToPoint({38_in, 65_in}, false, 1500, 1000);
  //odom::driveToPoint({0_in, 15_in}, true, 1000, 1000);
  //odom::driveToPoint({0_in, 0_in}, true, 1000, 1000);
  odom::driveToPoint({12_in, -2_in}, true, 1000, 1000);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
  conveyor.moveVelocity(200);
}
