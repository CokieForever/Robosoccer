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
        GO_TO_DEF_POS, FOLLOWPATH, STOP, DEFENSE
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
    void performCmd(const Interpreter::GameData& info);
    void defend_p2 (void);

  private:
    BallMonitor* m_ballpt;
    queue<int> m_q;
    string m_path;
    ActionPlayerTwo m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;
    Position m_defendpm;
    
    void AddObstacleForFormation(const Interpreter::GameData& info);


};
#endif // PLAYERTWO_H
