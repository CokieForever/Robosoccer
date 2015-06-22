#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "cruiseToBias2.h"
#include "coordinates.h"
#include "interpreter.h"
#include <queue>


class PlayerMain
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

    static void* performCmd_helper(void *context);

    PlayerMain(RoboControl *a, RawBall *b, CoordinatesCalibrer *c);

    int getMapValue(int i, int j) const;
    const Interpreter::Map& getMap() const;
    bool setMapValue(int i, int j, int val);
    void setMap(const Interpreter::Map &map0);

    RoboControl* getRobot() const;

    Position getDefaultPosition() const;
    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

    void setNextCmd(void *s);
    void setCmdParam();
    void* performCmd();

private:
    RawBall *m_ball;
    ActionPlayerMain m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;
    string m_path;
    queue<int> m_q;
    CoordinatesCalibrer *m_cal;
    bool m_actionPerformed;
    int m_go_x,m_go_y;
    Position m_defaultPos;
    RoboControl *m_robot;
    Interpreter::Map m_map;

};

#endif // PLAYERMAIN_H
