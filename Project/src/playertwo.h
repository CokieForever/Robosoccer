#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "interpreter.h"
#include <queue>


/**
 * @brief
 *
 */
class PlayerTwo : public TeamRobot
{

  public:
    /**
     * @brief
     *
     */
    enum ActionPlayerTwo
    {
        GO_TO_DEF_POS,  /**< TODO */
        FOLLOWPATH,     /**< TODO */
        STOP,           /**< TODO */
        DEFENSE       /**< TODO */
    };

    /**
     * @brief
     *
     */
    struct KickParam
    {
      double turnAngle;     /**< TODO */
      int force;            /**< TODO */
      Position pos;         /**< TODO */

    };

    PlayerTwo(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall*, BallMonitor *ballPm, RefereeDisplay *display = NULL);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(const Interpreter::GameData& info);

  private:
    static const double DEFENSE_LINE = 0.75;    /**< TODO */

    BallMonitor* m_ballpt;          /**< TODO */
    queue<int> m_q;                 /**< TODO */
    string m_path;                  /**< TODO */
    ActionPlayerTwo m_nextCmd;      /**< TODO */
    KickParam m_kickPenaltyParam;   /**< TODO */
    Position m_kickOffParam;        /**< TODO */
    Position m_defendpm;            /**< TODO */
    
    void AddObstacleForFormation(const Interpreter::GameData& info);

};
#endif // PLAYERTWO_H
