#ifndef CRUISETO_H
#define CRUISETO_H

#include "robo_control.h"
#include "kogmo_rtdb.hxx"

class RobotCruiseTo: public RoboControl
{
    public:
        enum eDirection
        {
            FORWARD, BACKWARD
        };

        RobotCruiseTo(RTDBConn& DBC, const int deviceNr);

        bool cruiseTo(double tarX, double tarY, double tarP);
        eDirection getDirection(double nominal, double actual);
        void setSpeed(double translation, double rotation, eDirection dir = FORWARD);
        double getDiffAngle(double nominal, double actual);
        double getSpeedP(double nominal, double actual);
        double getSpeedPt(double nominal, double actual);
        double getSpeedT(double diff);
        double degToRad(double deg);

};

#endif // CRUISETO_H
