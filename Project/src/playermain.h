#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include "interpreter.h"
#include <queue>


class PlayerMain : public TeamRobot
{

public:
    enum ActionPlayerMain
    {
        GO_TO_DEF_POS, KICK_PENALTY, KICK_OFF, STOP, FOLLOWPATH
    };

    struct KickParam
    {
        double  turnAngle;
        int  force;
        bool action1Performed,action2Performed;
        Position ball,pos;
    };

    PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *c, RawBall *b);

    void setNextCmd(Interpreter *info);
    void setCmdParam(void);
    void* performCmd(void);

private:
    ActionPlayerMain m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;
    string m_path;
    queue<int> m_q;
    bool m_actionPerformed;
    int m_go_x,m_go_y;

};

#endif // PLAYERMAIN_H
