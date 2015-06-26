
#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_


#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "teamrobot.h"
#include "coordinates.h"
#include "ballmonitor.h"


class Goalkeeper : public TeamRobot
{

  public:
    enum ActionGk
    {
      GO_TO_DEF_POS, PREVENT_GOAL, FOLLOWPATH
    };

    Goalkeeper(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer* coordCalib, RawBall*, BallMonitor* ballgk);

    void setNextCmd(Interpreter* info);
    void setCmdParam(void);
    void* performCmd(void);


  private:
    ActionGk m_nextCmd;
    Position m_preventGoalParam;
    BallMonitor* m_ballgk;
    Position m_defendgk;
    Position m_predictballgk;


};

#endif /* GOALKEEPER_H_ */
