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
      GO_TO_DEF_POS, KICK_PENALTY, KICK_OFF, STOP, FOLLOWPATH, DEFENSE, ATTACK
    };

    struct KickParam
    {
      double  turnAngle;
      int  force;
      Position pos;

    };

    PlayerTwo(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer* coordCalib, RawBall*, BallMonitor* ballpt);


    void setNextCmd(Interpreter* info);
    void setCmdParam(void);
    void* performCmd(void);
    void defend_p2 (void);

  private:
    BallMonitor* m_ballpt;
    Position m_defendp2;
    queue<int> m_q;
    string m_path;
    ActionPlayerTwo m_nextCmd;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;
    int m_go_x, m_go_y;


};
#endif // PLAYERTWO_H
