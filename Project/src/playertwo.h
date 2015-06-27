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
        GO_TO_DEF_POS, FOLLOWPATH, STOP
    };

    struct KickParam
    {
       double  turnAngle;
       int  force;
       Position pos;

    };

    PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall*, RefereeDisplay *display);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(void);

private:
    ActionPlayerTwo m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;

    void AddObstacleForFormation(Interpreter::Strategy formation);

};
#endif // PLAYERTWO_H
