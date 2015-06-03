#ifndef CRUISETOBIAS_H
#define CRUISETOBIAS_H

#include "robo_control.h"
#include "kogmo_rtdb.hxx"

class RobotCruisetoBias: public RoboControl
{
public:
    enum eDirection
    {
      FORWARD,
      BACKWARD
    };

    RobotCruisetoBias(RTDBConn& DBC, const int deviceNr);

    bool cruisetoBias(double tarX, double tarY, int speed, double tarP, double varDir);
    eDirection getDirection(double nominal, double actual);
    void setSpeed(double translation, double rotation, eDirection dir);
    double getDiffAngle(double nominal, double actual);
    double getSpeedP(double nominal, double actual); // Drehgeschwindigkeit ruhig
    double getSpeedPt(double nominal, double actual, int geschw); // Drehgeschwindigkeit bei der Fahrt
    double getSpeedT(double diff);
    double degToRad(double deg); // regelt Vorwärtsgeschwindigkeit

private:
    void StopMovement(void);
};

#endif // CRUISETOBIAS_H
