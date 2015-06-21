#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "cruiseToBias2.h"
#include "coordinates.h"
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
    queue<int> m_q;
    CoordinatesCalibrer *cal;
    bool actionPerformed;
    int go_x,go_y;



public:
    Position defaultPos;
    RoboControl* robot;
    int map[WIDTH][HEIGHT];

    void setNextCmd(void*);
    void setCmdParam();
    void *performCmd();
    static void *performCmd_helper(void *);

    PlayerMain(RoboControl*a,RawBall *b,CoordinatesCalibrer *c);
    ~PlayerMain();

};

#endif // PLAYERMAIN_H
