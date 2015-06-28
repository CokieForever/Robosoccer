#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include <ballmonitor.h>


class PlayerMain : public TeamRobot
{

  public:
    enum ActionPlayerMain
    {
      GO_TO_DEF_POS, KICK_PENALTY, KICK_OFF, STOP, FOLLOWPATH, DEFENSE, ATTACK
    };


    struct KickParam
    {
      double  turnAngle;
      int  force;
      bool action1Performed, action2Performed;
      Position ball, pos;
    };

    PlayerMain(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *c, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display = NULL);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(void);

  private:
    ActionPlayerMain m_nextCmd;
    BallMonitor* m_ballpm;
    Position m_defendpm;
    KickParam m_kickPenaltyParam;
    Position m_kickOffParam;

    void AddObstacleForFormation(Interpreter::Strategy formation);

};

#endif // PLAYERMAIN_H
