#ifndef PLAYERMAIN_H
#define PLAYERMAIN_H

#include "kogmo_rtdb.hxx"
#include "teamrobot.h"
#include "coordinates.h"
#include "interpreter.h"
#include <queue>


/**
 * @brief
 *
 */
class PlayerMain : public TeamRobot
{

public:
    /**
     * @brief
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

    void setNextCmd(Interpreter *info);
    void setCmdParam(void);
    void* performCmd(void);

private:
    ActionPlayerMain m_nextCmd;             /**< TODO */
    BallMonitor* m_ballpm;                  /**< TODO */
    const NewRoboControl* m_otherRobots[5]; /**< TODO */
    Position m_defendp2;                    /**< TODO */
    string m_path;
    queue<int> m_q;
    bool m_actionPerformed;
    int m_go_x,m_go_y;

    void AddObstacleForFormation(const Interpreter::GameData& info);
    void defend_p2(void);

};

#endif // PLAYERMAIN_H
