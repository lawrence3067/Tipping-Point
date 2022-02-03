#include "main.h"

using namespace okapi;

namespace odom
{

  QVector<QLength> pos;
	QVector<Number> heading;
	QAngle curAngle = 90_deg; // for the sake of speed (technically could just use heading, no accuracy loss)
  QAngle turnAngle;

	double rPrev;
	double lPrev;
	double mPrev;
  int timer;
  double inertialVal;
  double targetAngle1;

  typedef struct PID pid;
  pid rotate;
  pid translate;

  void update()
  {
    QLength distL = 2_in * (drive -> getModel() -> getSensorVals()[0] * 18 / 35 - lPrev) / 180.0 * PI; //convert to radians to get distance that wheel has turned
    QLength distR = 2_in * (drive -> getModel() -> getSensorVals()[1] * 18 / 35 - rPrev) / 180.0 * PI;

    QLength forward = (distL + distR) / 2;
    //QAngle dtheta = (distR - distL) / 13.7_in * radian; // small angle

    //update pos
    pos += forward * heading;
    if (inertialSensor.get() < -90 && inertialSensor.get() > -180)
    {
      curAngle = (90 - (inertialSensor.get() + 360)) * PI / 180 * radian;
    }
    else
    {
      curAngle = (90 - inertialSensor.get()) * PI / 180 * radian;
    }

  	heading = QVector<Number>(curAngle); // change heading (apply curAngle)
    lPrev = drive -> getModel() -> getSensorVals()[0] * 18 / 35;
  	rPrev = drive -> getModel() -> getSensorVals()[1] * 18 / 35;
  }

  void driveToPoint(Point target, bool driveBack, int ms)
  {
    QVector<QLength> difference = QVector(target.x, target.y) - pos;
    if (driveBack == false)
    {
      turnAngle = difference.arg();  //returns direction of vector
    }
    else
    {
      turnAngle = (difference.arg().convert(radian) + PI) * radian; //angled so the back of the bot faces coordinate
    }
    QLength targetDistance = difference.norm(); //returns magnitude of vector

    odomRotate(turnAngle);
    odomTranslate(targetDistance, driveBack, ms);
  }

  void odomRotate(QAngle targetAngle)
  {
    targetAngle1 = targetAngle.convert(radian) * 180 / PI; //convert from radian to double

    rotate.kP = 0.032;
    rotate.kI = 0;
    rotate.kD = 0.0009;

    auto rotateController = IterativeControllerFactory::posPID(rotate.kP, rotate.kI, rotate.kD);

    while (true)
    {
      pros::lcd::set_text(3, std::to_string(inertialVal));
      if (inertialSensor.get() < -90 && inertialSensor.get() > -180) //checks if inertial reading is in 3rd quadrant
      {
        inertialVal = 90 - (inertialSensor.get() + 360); //remaps inertial readings to have 0 @ east, PI @ west, -PI/2 @ south
      }
      else
      {
        inertialVal = 90 - inertialSensor.get();
      }

      rotate.power = rotateController.step(targetAngle1 - inertialVal);

      drive -> getModel() -> tank(rotate.power, -rotate.power);
      update();

      if (abs(targetAngle1 - inertialVal) < 2 && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
        && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1 && leftTop.getActualVelocity() < 1
        && leftBottom.getActualVelocity() < 1)
      {
        break;
      }
      pros::delay(10); 
    }

    drive -> getModel() -> tank(0, 0);
  }

  void odomTranslate(QLength targetDistance, bool driveBack, int ms)
  {
    double targetDegrees = (targetDistance.convert(inch) * 90 / PI) * 35/18;

    if (driveBack == false)
    {
      translate.target = drive -> getModel() -> getSensorVals()[0] + targetDegrees;
    }
    else if (driveBack == true)
    {
      translate.target = drive -> getModel() -> getSensorVals()[0] - targetDegrees;
    }

    timer = 0;

    translate.kP = 0.001;
  	translate.kI = 0;
  	translate.kD = 0.000025;

  	auto translateController = IterativeControllerFactory::posPID(translate.kP, translate.kI, translate.kD);

  	while (timer < ms)
  	{
  		translate.error = translate.target - drive -> getModel() -> getSensorVals()[0];
  		translate.power = translateController.step(translate.error);

      drive -> getModel() -> tank(-translate.power, -translate.power);

  		if (abs(translate.error) < 6  && rightFront.getActualVelocity() < 1 && rightTop.getActualVelocity() < 1
        && rightBottom.getActualVelocity() < 1 && leftFront.getActualVelocity() < 1
        && leftTop.getActualVelocity() < 1 && leftBottom.getActualVelocity() < 1)
  		{
  			break;
  		}

      timer += 10;
  		pros::delay(10);
  	}

  	drive -> getModel() -> tank(0, 0);
    update();
  }
}
