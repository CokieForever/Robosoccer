#include "teamrobot.h"

void* TeamRobot::performCmd_helper(void *context)
{
    return ((TeamRobot*)context)->performCmd();
}

TeamRobot::TeamRobot(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *ball, RefereeDisplay *display) : NewRoboControl(DBC, deviceNr)
{
    m_coordCalib = coordCalib;
    m_ball = ball;
    m_display = display;
    memset(&(m_map[0][0]), 0, sizeof(int)*Interpreter::MAP_WIDTH*Interpreter::MAP_HEIGHT);
    m_pathFinder.AddRectangle(PathFinder::CreatePoint(-0.3, -0.3), PathFinder::CreatePoint(0.3, 0.3));
}

Position TeamRobot::getDefaultPosition() const
{
    return m_defaultPos;
}

void TeamRobot::setDefaultPositionX(double x)
{
    m_defaultPos.SetX(x);
}

void TeamRobot::setDefaultPositionY(double y)
{
    m_defaultPos.SetY(y);
}

void TeamRobot::setDefaultPosition(Position pos)
{
    m_defaultPos = pos;
}

int TeamRobot::getMapValue(int i, int j) const
{
    return (i<Interpreter::MAP_WIDTH && j<Interpreter::MAP_HEIGHT && i>0 && j>0) ? m_map[i][j] : 0;
}

const Interpreter::Map& TeamRobot::getMap() const
{
    return m_map;
}

bool TeamRobot::setMapValue(int i, int j, int val)
{
    if (i<Interpreter::MAP_WIDTH && j<Interpreter::MAP_HEIGHT && i>0 && j>0)
    {
        m_map[i][j] = val;
        return true;
    }
    return false;
}

void TeamRobot::setMap(const Interpreter::Map &map)
{
    memcpy(&(m_map[0][0]), &(map[0][0]), sizeof(int)*Interpreter::MAP_WIDTH*Interpreter::MAP_HEIGHT);
}

