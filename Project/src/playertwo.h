#ifndef PLAYERTWO_H
#define PLAYERTWO_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "interpreter.h"
#include <queue>


/**
 * @brief  Child class of @ref TeamRobot which describes the second player.
 *
 * The game behavior of the second player is slightly different from its brother @ref PlayerMain "player main".
 */
class PlayerTwo : public TeamRobot
{

  public:
    /**
     * @brief Enumeration of the possible orders for the player two.
     *
     */
    enum ActionPlayerTwo
    {
        GO_TO_DEF_POS,  /**< Go to the default position */
        FOLLOWPATH,     /**< Use the path planning */
        STOP,           /**< Stop */
        DEFENSE       /**< Protect the first defense line */
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
    Position m_kickOffParam;        /**< TODO */
    Position m_defendpm;            /**< TODO */
    
    void AddObstacleForFormation(const Interpreter::GameData& info);

};
#endif // PLAYERTWO_H
