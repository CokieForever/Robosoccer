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
        GO_TO_DEF_POS, PREVENT_GOAL, STOP
    };

    Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall*);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(void);

private:
    ActionGk m_nextCmd;
    Position m_preventGoalParam;
    BallMonitor* m_ballgk;
    Position m_defendgk;
    Position m_predictballgk;
    
    void AddObstacleForFormation(Interpreter::Strategy formation);

};

#endif /* GOALKEEPER_H_ */
