#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"


class PlayerTwo : public TeamRobot
{

public:
    enum ActionPlayerTwo
    {
        GO_TO_DEF_POS, KICK_PENALTY, KICK_OFF, PLAY, STOP
    };

    struct KickParam
    {
       double  turnAngle;
       int  force;
       Position pos;

    };

    PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall*);

    void setNextCmd(Interpreter *info);
    void setCmdParam(void);
    void* performCmd(void);

private:
    ActionPlayerTwo m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;

};
#endif // PLAYERTWO_H
