#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include <ballmonitor.h>


/**
 * @brief Child class of @ref TeamRobot which describes the main player.
 *
 * The main player is responsible for kick-off and penalty shootings, and its game behavior is slightly different
 * from its brother @ref PlayerTwo "player two".
 */
class PlayerMain : public TeamRobot
{

  public:
    /**
     * @brief Enumeration of the possible orders for the player.
     *
     */
    enum ActionPlayerMain
    {
        GO_TO_DEF_POS,  /**< TODO */
        KICK_PENALTY,   /**< TODO */
        KICK_OFF,       /**< TODO */
        STOP,           /**< TODO */
        FOLLOWPATH      /**< TODO */
    };

    PlayerMain(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *c, RawBall *b, BallMonitor *ballPm, RefereeDisplay *display = NULL);

    void setNextCmd(const Interpreter::GameData& info);
    void setCmdParam(const Interpreter& interpreter);
    void performCmd(const Interpreter::GameData& info);

  private:
    ActionPlayerMain m_nextCmd;             /**< TODO */
    BallMonitor* m_ballpm;                  /**< TODO */
    const NewRoboControl* m_otherRobots[5]; /**< TODO */
    Position m_defendp2;                    /**< TODO */

    void AddObstacleForFormation(const Interpreter::GameData& info);
    void defend_p2(void);

};

#endif // PLAYERMAIN_H
