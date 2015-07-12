
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
    /**
     * @brief
     *
     */
    enum ActionGk
    {
        GO_TO_DEF_POS,  /**< Go to default position */
        PREVENT_GOAL,   /**< Prevent goal */
        STOP            /**< stop moving */
    };

    Goalkeeper(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer* coordCalib, RawBall* ball, BallMonitor* ballgk);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(const Interpreter::GameData& info);


  private:
    ActionGk m_nextCmd;             /**< Next command for the goalkeeper */
    Position m_preventGoalParam;    /**< The position to prevent the goal */
    BallMonitor* m_ballgk;          /**< The monitor of ball */
    
    void AddObstacleForFormation(const Interpreter::GameData& info);


};

#endif /* GOALKEEPER_H_ */
