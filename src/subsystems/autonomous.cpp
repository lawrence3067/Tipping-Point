#include "main.h"

using namespace okapi;

void awpAuton()
{
  //route 2: awp side
  itz_lift.moveVelocity(itzPID(330)); //lift down
  pros::delay(10);
  pros::c::adi_digital_write(pneumaticLeftPort, LOW);
  translatePID(-100, -100);
  pros::c::adi_digital_write(pneumaticLeftPort, HIGH);
  pros::delay(500);
  itz_lift.moveVelocity(itzPID(50)); //lift up
  pros::delay(500);
  rotate_PID(-115);
  pros::c::adi_digital_write(pneumaticLeftPort, LOW);
  itz_lift.moveVelocity(itzPID(330));
  translatePID(-45, -45); //-35
  translatePID(35, 35);
  rotate_PID(78);
  translatePID(-50, -50);
  pros::c::adi_digital_write(pneumaticLeftPort, HIGH);
  pros::delay(500);
  itz_lift.moveVelocity(50);
  translatePID(100, 100);
  /**
  //four bar should be facing forward
  translatePID(10,10);
  fourBarLift.moveVelocity(100);//tune //pick up center mogo
  translatePID(-20,-20); //move back
  rotate_PID(15);  //use itz lift to pick up awp mogo
  itz_lift.moveVelocity(330); //lift down
  translatePID(10,10); //move forward to pick up mogo
  itz_lift.moveVelocity(50); //lift up
  translatePID(-20,-20); //move mogo out of AWP zone
  rollers.moveVelocity(200); //drop rings into mogo
  //by nidhi 👍 wiat you can use emojis in this
  **/

}
