#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "interpreter.h"
#include <queue>


class PlayerTwo : public TeamRobot
{

  public:
    enum ActionPlayerTwo
    {
        GO_TO_DEF_POS, FOLLOWPATH, STOP
    };

    struct KickParam
    {
      double  turnAngle;
      int  force;
      Position pos;

    };

    PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall*, BallMonitor *ballPm, RefereeDisplay *display = NULL);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(void);
    void defend_p2 (void);

  private:
    BallMonitor* m_ballpt;
    Position m_defendp2;
    queue<int> m_q;
    string m_path;
    ActionPlayerTwo m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;
    Position m_defendpm;
    
    void AddObstacleForFormation(Interpreter::Strategy formation);


};
#endif // PLAYERTWO_H
