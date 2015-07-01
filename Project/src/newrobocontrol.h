#ifndef NEWROBOCONTROL_H
#define NEWROBOCONTROL_H

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
#include <vector>

class NewRoboControl : public RoboControl
{

public:
    enum eDirection
    {
        FORWARD,
        BACKWARD
    };

    static eDirection getDirection(double nominal, double actual);
    static double getDiffAngle(double nominal, double actual);
    static double getSpeedP(double nominal, double actual); // Drehgeschwindigkeit ruhig
    static double getSpeedPt(double nominal, double actual, int geschw); // Drehgeschwindigkeit bei der Fahrt
    static double getSpeedT(double diff);
    static double degToRad(double deg); // regelt Vorwärtsgeschwindigkeit
    static bool IsOnTarget(Position current, Position target);

    NewRoboControl(RTDBConn& DBC, const int deviceNr);
    virtual ~NewRoboControl() = 0;  //Prevents instantiation of NewRoboControl

    bool IsOnTarget(Position target);
    bool cruisetoBias(double tarX, double tarY, int speed, double tarP, double varDir);
    Position* drivePath(std::vector<Position>* path);
    void setSpeed(double translation, double rotation, eDirection dir);

private:
    bool m_stopCruisingNow;
    bool m_isCruising;
    pthread_t m_cruiseThread;

};


#endif /* NEWROBOCONTROL_H */
