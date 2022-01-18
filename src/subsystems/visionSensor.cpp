#include "main.h"

using namespace okapi;

pros::Vision visionSensor(visionPort);

//color signatures
// vision::signature YELLOWMOGO (1, 1639, 2465, 2052, -4819, -4561, -4690, 5.700, 1);
// vision::signature REDMOGO (2, 10533, 11315, 10924, -1971, -1509, -1740, 4.200, 1);
// vision::signature BLUEMOGO (3, -3165, -2305, -2735, 9207, 10443, 9825, 5.400, 1);

int yellowSig = 1;
int redSig = 2;
int blueSig = 3;
int xCoord;
int yCoord;
int topCoord;
int avgX;

pros::vision_signature_s_t YELLOWMOGO =
pros::Vision::signature_from_utility(yellowSig, 2419, 5247, 3833, -4915, -4531, -4723, 4.400, 1);
// pros::vision_signature_s_t REDMOGO =
// pros::Vision::signature_from_utility(redSig, 10533, 11315, 10924, -1971, -1509, -1740, 4.200, 1);
// pros::vision_signature_s_t BLUEMOGO =
// pros::Vision::signature_from_utility(blueSig, -3165, -2305, -2735, 9207, 10443, 9825, 5.400, 1);

void updateVision()
{

  visionSensor.clear_led();
  pros::vision_object_s_t rtn = visionSensor.get_by_sig(0, yellowSig); //largest box
  //pros::lcd::set_text(2, std::to_string(rtn.signature));
  xCoord = rtn.x_middle_coord;
  yCoord = rtn.y_middle_coord;
  topCoord = rtn.top_coord; //top boundary coordinate
  pros::lcd::set_text(2, std::to_string(topCoord));


  if(topCoord > 182)
  {
    pros::vision_object_s_t rtn2 = visionSensor.get_by_sig(1, yellowSig);  //2nd largest box
    avgX = (rtn.x_middle_coord + rtn2.x_middle_coord)/2;
    //check if top coordinate of the mogo is under y line and that the middle is in some  x range
    pros::lcd::set_text(3, std::to_string(rtn.x_middle_coord));
    pros::lcd::set_text(4, std::to_string(rtn2.x_middle_coord));

    pros::lcd::set_text(5, std::to_string(avgX));
    if(avgX > 160 && avgX < 220){
      pros::c::adi_digital_write(fourBarClampPort, HIGH); //clamp down
      pros::lcd::set_text(3, "Hi");

    }
  }
  // else
  // {
  //   //move towards mogo because only one object detected
  //
  //   //pros::c::adi_digital_write(fourBarClampPort,HIGH); //clamp down
  // }
}
