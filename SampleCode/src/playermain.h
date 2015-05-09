#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "interpreter.h"
#include "share.h"

class PlayerMain
{
    enum ActionPlayerMain{GO_TO_DEF_POS,KICK_PENALTY,KICK_OFF}; // ADD PLAY ?
    struct KickParam
    {
       double  turnAngle;
       int  force;
       Postition pos;

    };

private:

    KickParam kickPenaltyParam;
    Rawball* ball;
    Robocontrol* player;
    ActionPlayerMain nextCmd;
    Position kickOffParam;


public:
    Position defaultPos;

    void setNextCmd(Interpreter*);
    void setCmdParam();
    void performCmd();
    PlayerMain();
    ~PlayerMain();

};

#endif // PLAYERMAIN_H
