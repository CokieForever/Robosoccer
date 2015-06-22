#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_


#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"


class Goalkeeper {

public:
    enum ActionGk
    {
        GO_TO_DEF_POS, PREVENT_GOAL, FOLLOWPATH
    };

    static void *performCmd_helper(void *);

    Goalkeeper(RoboControl*,RawBall*);

    void setNextCmd(void*);
    void setCmdParam();
    void *performCmd(void);

    RoboControl* getRobot();
    Position getDefaultPosition();

    void setDefaultPositionX(double x);
    void setDefaultPositionY(double y);
    void setDefaultPosition(Position pos);

private:
    RawBall* m_ball;

    ActionGk m_nextCmd;
    Position m_preventGoalParam;

    Position m_defaultPos;
    RoboControl* m_robot;

};

#endif /* GOALKEEPER_H_ */
