#ifndef OPPONENTROBOT_H
#define OPPONENTROBOT_H

#include "newrobocontrol.h"

/**
 * @brief
 *
 */
class OpponentRobot : public NewRoboControl
{

public:
    OpponentRobot(RTDBConn& DBC, const int deviceNr);

private:
    /* Well, nothing. */

};

#endif // OPPONENTROBOT_H
