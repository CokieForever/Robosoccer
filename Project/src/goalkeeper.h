
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
        GO_TO_DEF_POS,  /**< TODO */
        CLEAR_GOAL,     /**< TODO */
        PREVENT_GOAL,   /**< TODO */
        STOP            /**< TODO */
    };

    Goalkeeper(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer* coordCalib, RawBall* ball, BallMonitor* ballgk);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(const Interpreter::GameData& info);


  private:
    ActionGk m_nextCmd;             /**< TODO */
    Position m_preventGoalParam;    /**< TODO */
    BallMonitor* m_ballgk;          /**< TODO */
    
    void AddObstacleForFormation(const Interpreter::GameData& info);


};

#endif /* GOALKEEPER_H_ */
