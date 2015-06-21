#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include <queue>


class PlayerMain
{
    enum ActionPlayerMain{GO_TO_DEF_POS,KICK_PENALTY,KICK_OFF,STOP,FOLLOWPATH};
    struct KickParam
    {
       double  turnAngle;
       int  force;
       bool action1Performed,action2Performed;
       Position ball,pos;


    };

private:
    RawBall* ball;
    ActionPlayerMain nextCmd;
    KickParam kickPenaltyParam;
    Position kickOffParam;
    string path;
    queue<Position> q;
    bool actionPerformed;



public:
    Position defaultPos;
    RoboControl* robot;
    int map[WIDTH][HEIGHT];

    void setNextCmd(void*);
    void setCmdParam();
    void *performCmd();
    static void *performCmd_helper(void *);

    PlayerMain(RoboControl*,RawBall *);
    ~PlayerMain();

};

#endif // PLAYERMAIN_H
