#include "main.h"

double difference;

void rightLine()
{
  leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

  rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);
  inertialSensor.reset();
  inertialSensor2.reset();

  fourBarClamp.set_value(true);
  tilter.set_value(true);
  tilterClamp.set_value(false);
  translatePID(60, 2000, true);
  translatePID(-28, 1500, false);
  difference = -90 - inertialSensor2.get();
  rotatePID(difference, 1000);
  translatePID(-30, 1000, false);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
  translatePID(12, 700, false);
  fourBarLift.moveVelocity(200);
  difference = 92 - inertialSensor2.get();
  rotatePID(difference, 1000);
  conveyor.moveVelocity(600);
  translateJank(30, 5000, false);
  tilter.set_value(true);
  translatePID(-40, 1000, false);
}

void middleStar()
{
  leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

  rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);
  inertialSensor.reset();
  inertialSensor2.reset();

  fourBarClamp.set_value(true);
  tilter.set_value(true);
  tilterClamp.set_value(false);
  /**
  translatePID(70, 2000, true);
  translatePID(-33, 2000, false);
  difference = -55 - inertialSensor2.get();
  rotatePID(difference, 1000);
  translatePID(-35, 2000, false);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
  fourBarLift.moveVelocity(200);
  difference = -45 - inertialSensor2.get();
  rotatePID(difference, 1000);
  conveyor.moveVelocity(600);
  translateJank(20, 5000, false);**/
  odom::driveToPoint({-35_in, 48_in}, false, 1500, 1000, true);
  odom::driveToPoint({-20_in, 19_in}, true, 1000, 1000, false);
  conveyor.moveVelocity(600);
  odom::driveToPoint({20_in, 19_in}, true, 2000, 1000, false);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
  fourBarLift.moveVelocity(200);
  conveyor.moveVelocity(600);
  rotatePID(-55, 1000);
  translateJank(35, 3000, false);
  translatePID(-15, 1000, false);
  translateJank(15, 3000, false);
  translatePID(-15, 1000, false);
  translateJank(15, 3000, false);

}
void leftAutonWP()
{
  leftFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftBottom.setBrakeMode(AbstractMotor::brakeMode::hold);

  rightFront.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightBottom.setBrakeMode(AbstractMotor::brakeMode::hold);
  inertialSensor.reset();
  inertialSensor2.reset();

  fourBarClamp.set_value(true);
  tilter.set_value(true);
  tilterClamp.set_value(false);

  translatePID(65, 2000, true);
  pros::delay(100);
  translatePID(-47, 2000, false);
  rotatePID(-77, 1000);
  translatePID(-15, 1000, false);
  conveyor.moveVelocity(600);
  pros::delay(1000);
  tilterClamp.set_value(true);
  pros::delay(500);
  tilter.set_value(false);
}
