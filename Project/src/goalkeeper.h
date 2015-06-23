#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_


#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "teamrobot.h"
#include "coordinates.h"


class Goalkeeper : public TeamRobot
{

public:
    enum ActionGk
    {
        GO_TO_DEF_POS, PREVENT_GOAL, FOLLOWPATH
    };

    Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall*);

    void setNextCmd(Interpreter *info);
    void setCmdParam(void);
    void* performCmd(void);

private:
    ActionGk m_nextCmd;
    Position m_preventGoalParam;

};

#endif /* GOALKEEPER_H_ */
