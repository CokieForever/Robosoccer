#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "robo_control.h"


class PlayerTwo
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

    static void* performCmd_helper(void *context);

    PlayerTwo(RoboControl*,RawBall *);

    RoboControl* getRobot() const;
    Position getDefaultPosition() const;

    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

    void setNextCmd(void *s);
    void setCmdParam();
    void* performCmd();

private:
    RawBall* ball;
    ActionPlayerTwo nextCmd;

    KickParam kickPenaltyParam;
    Position kickOffParam;

    Position defaultPos;
    RoboControl* robot;

};
#endif // PLAYERTWO_H
