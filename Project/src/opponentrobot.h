#ifndef OPPONENTROBOT_H
#define OPPONENTROBOT_H

#include "newrobocontrol.h"

/**
 * @brief Child class of @ref NewRoboControl which desribes a robo of the other team.
 *
 * This class does not implement anything and is just used to make the distinction with a @ref TeamRobot.
 * It is however still possible to use the driving functions of the parents @ref NewRoboControl and "RoboControl",
 * but this should be avoided as it is forbidden by the game rules.
 */
class OpponentRobot : public NewRoboControl
{

public:
    OpponentRobot(RTDBConn& DBC, const int deviceNr);

private:
    /* Well, nothing. */

};

#endif // OPPONENTROBOT_H
