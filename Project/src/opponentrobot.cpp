#include "opponentrobot.h"

/**
 * @brief Basic constructor
 *
 * @param DBC
 * @param deviceNr
 */
OpponentRobot::OpponentRobot(RTDBConn& DBC, const int deviceNr) : NewRoboControl(DBC, deviceNr)
{
}
