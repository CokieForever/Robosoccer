#ifndef cruise2_CPP_
#define cruise2_CPP_

//#include "pololu_high_level_control.h"

#include "pololu_status.h"
#include "pololu_cmd.h"
#include "cam.h"
#include "raw_ball.h"
#include "angle.h"
#include "position.h"
#include "share.h"
#include "types.h"
#include "vector3d.h"

#include "robo_control.h"

enum eDirection
{
  FORWARD,
  BACKWARD
};

bool CruisetoBias(double tarX, double tarY, int speed, double tarP, double varDir, RoboControl * robo);
eDirection getDirection(double nominal, double actual);
    void setSpeed(double translation, double rotation, eDirection dir, RoboControl * robo);
    double getDiffAngle(double nominal, double actual);
    double getSpeedP(double nominal, double actual); // Drehgeschwindigkeit ruhig
    double getSpeedPt(double nominal, double actual, int geschw); // Drehgeschwindigkeit bei der Fahrt
    double getSpeedT(double diff);
    double degToRad(double deg); // regelt Vorwärtsgeschwindigkeit
    inline void StopMovement(void);




#endif /* cruise2_CPP_ */
