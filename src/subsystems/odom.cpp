#include "main.h"
//#include "QVector.hpp"

namespace odom
{

  QVector<QLength> pos;
	QVector<Number> heading;
	QAngle curAngle = 90_deg; // for the sake of speed (technically could just use heading, no accuracy loss)

	double rPrev;
	double lPrev;
	double mPrev;

  typedef struct PID pid;
  pid rotate;
  pid move;

  void update()
  {
    QLength distL = 2_in * (drive -> getModel() -> getSensorVals()[0] * 18 / 35 - lPrev) / 180.0 * PI; // encoders measure things in degrees, so first convert to radians to get the distance that wheel has turned
    QLength distR = 2_in * (drive -> getModel() -> getSensorVals()[1] * 18 / 35 - rPrev) / 180.0 * PI;

    QLength forward = (distL + distR) / 2;
    //QAngle dtheta = (distR - distL) / 13.7_in * radian; // small angle

    //update pos
    pos += forward * heading;
    curAngle = inertialSensor.get(); // change angle

  	heading = QVector<Number>(curAngle); // heading also changes
    lPrev = drive -> getModel() -> getSensorVals()[0] * 18 / 35;
  	rPrev = drive -> getModel() -> getSensorVals()[1] * 18 / 35;
  }

  void driveToPoint(Point target, bool driveBack)
  {
    QVector<QLength> difference = QVector(target.x, target.y) - pos;
    QAngle turnAngle = difference.arg();  //returns direction of vector
    QLength targetDistance = difference.norm(); //returns magnitude of vector

    odomRotate(turnAngle);
    odomTranslate(targetDistance, driveBack);
  }

  void odomRotate(QAngle targetAngle)
  {
    QAngle inertialVal = (90 - inertialSensor.get()) * degree;
    targetAngle = targetAngle.convert(radian) * 180 / PI * degree;

    rotate.kP = 0.032;
    rotate.kI = 0;
    rotate.kD = 0.001;

    auto rotateController = IterativeControllerFactory::posPID(rotate.kP, rotate.kI, rotate.kD);

    while (abs(targetAngle - inertialVal) >= 2_deg)
    {
      pros::lcd::set_text(3, std::to_string(inertialVal.convert(degree)));
      inertialVal = (90 - inertialSensor.get()) * degree;
      rotate.power = rotateController.step((targetAngle - inertialVal).convert(degree));

      drive -> getModel() -> tank(rotate.power, -rotate.power);
      update();

      pros::delay(10);
    }

    drive -> getModel() -> tank(0, 0);
  }
/**
  void odomTranslate(Point target)
  {
    move.kP = 0;
    move.kI = 0;
    move.kD = 0;

    auto targetVector = QVector(target.x, target.y);

    QVector<QLength> difference = targetVector - pos;
    auto moveController = IterativeControllerFactory::posPID(move.kP,  move.kI, move.kD);

    while (difference.norm() >= 1_in)
    {
      difference = targetVector - pos;
      move.power = moveController.step(difference.norm().convert(inch));

      drive -> getModel() -> tank(-move.power, -move.power);
      update();

      pros::delay(10);
    }

    drive -> getModel() -> tank(0, 0);
  }**/
  void odomTranslate(QLength targetDistance, bool driveBack)
  {
    double targetDegrees = (targetDistance.convert(inch) * 90 / PI) * 35/18;
    if (driveBack == true)
    {
      drive -> moveRaw(-targetDegrees);
    }
    else
    {
      drive -> moveRaw(targetDegrees);
    }
    update();
  }
}

