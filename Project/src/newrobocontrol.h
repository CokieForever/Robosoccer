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

/**
 * @brief Wrapper for the "RoboControl" class. Implements additional driving functions.
 *
 * In order to avoid confusion, this class was made abstract.
 */
class NewRoboControl : public RoboControl
{

public:
    /**
     * @brief Enumeration for the driving directions.
     *
     */
    enum eDirection
    {
        FORWARD,    /**< TODO */
        BACKWARD    /**< TODO */
    };

    static eDirection getDirection(double nominal, double actual);
    static double getDiffAngle(double nominal, double actual);
    static double getSpeedP(double nominal, double actual); // Drehgeschwindigkeit ruhig
    static double getSpeedPt(double nominal, double actual, int geschw); // Drehgeschwindigkeit bei der Fahrt
    static double getSpeedT(double diff);
    static double degToRad(double deg); // regelt Vorwärtsgeschwindigkeit
    static bool IsOnTarget(Position current, Position target, bool precise=true);

    NewRoboControl(RTDBConn& DBC, const int deviceNr);
    virtual ~NewRoboControl() = 0;  //Prevents instantiation of NewRoboControl

    bool IsOnTarget(Position target, bool precise=true) const;
    bool cruisetoBias(double tarX, double tarY, int speed, double tarP=-10, double varDir=30, double dist=0);
    void RandomMove();
    bool drivePath(std::vector<Position>* path);
    void setSpeed(double translation, double rotation, eDirection dir);

private:
    bool m_stopCruisingNow;     /**< TODO */
    bool m_isCruising;          /**< TODO */
    pthread_t m_cruiseThread;   /**< TODO */

};


#endif /* NEWROBOCONTROL_H */
