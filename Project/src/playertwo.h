#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "robo_control.h"


class PlayerTwo
{
    enum ActionPlayerTwo{GO_TO_DEF_POS,KICK_PENALTY,KICK_OFF,PLAY,STOP};
    struct KickParam
    {
       double  turnAngle;
       int  force;
       Position pos;

    };

private:
    RawBall* ball;
    ActionPlayerTwo nextCmd;

    KickParam kickPenaltyParam;
    Position kickOffParam;


public:
    Position defaultPos;
    RoboControl* robot;

    void setNextCmd(void*);
    void setCmdParam();
    void *performCmd();
    static void *performCmd_helper(void *);
    PlayerTwo(RoboControl*,RawBall *);
    ~PlayerTwo();

};
#endif // PLAYERTWO_H
