#include "teamrobot.h"
#include "sdlutilities.h"

void* TeamRobot::performCmd_helper(void *context)
{
    return ((TeamRobot*)context)->performCmd();
}

TeamRobot::TeamRobot(RTDBConn& DBC, const int deviceNr, CoordinatesCalibrer *coordCalib, RawBall *ball, RefereeDisplay *display) : NewRoboControl(DBC, deviceNr)
{
    m_coordCalib = coordCalib;
    m_ball = ball;
    m_display = display;
    m_pathFinderPath = NULL;
    memset(&(m_map[0][0]), 0, sizeof(int)*Interpreter::MAP_WIDTH*Interpreter::MAP_HEIGHT);

    m_ballObstaclePos = Position(-10, -10);
    for (int i=0 ; i < 3 ; i++)
        m_ballObstacles[i] = NULL;
    for (int i=0 ; i < 5 ; i++)
    {
        m_roboObstacles[i] = NULL;
        m_roboObstaclePos[i] = Position(-10, -10);
    }

    //Left penalty area
    PathFinder::Point ul = PathFinder::CreatePoint(-1, -0.25);
    PathFinder::Point lr = PathFinder::CreatePoint(-0.9, 0.25);
    m_pathFinder.AddRectangle(ul, lr);

    //Right penalty area
    ul = PathFinder::CreatePoint(0.9, -0.25);
    lr = PathFinder::CreatePoint(1, 0.25);
    m_pathFinder.AddRectangle(ul, lr);

    //Upper border
    ul = PathFinder::CreatePoint(-1, -1);
    lr = PathFinder::CreatePoint(1, -0.95);
    m_pathFinder.AddRectangle(ul, lr);

    //Lower border
    ul = PathFinder::CreatePoint(-1, 0.95);
    lr = PathFinder::CreatePoint(1, 1);
    m_pathFinder.AddRectangle(ul, lr);

    //Left border
    ul = PathFinder::CreatePoint(-1, -0.95);
    lr = PathFinder::CreatePoint(-0.95, 0.95);
    m_pathFinder.AddRectangle(ul, lr);

    //Right border
    ul = PathFinder::CreatePoint(0.95, -0.95);
    lr = PathFinder::CreatePoint(1, 0.95);
    m_pathFinder.AddRectangle(ul, lr);
}

TeamRobot::~TeamRobot()
{
    if (m_pathFinderPath)
        delete m_pathFinderPath;
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

CoordinatesCalibrer* TeamRobot::getCoordinatesCalibrer() const
{
    return m_coordCalib;
}

RawBall* TeamRobot::getBall() const
{
    return m_ball;
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

void TeamRobot::UpdatePathFinder(NewRoboControl *const obstacles[5], eSide our_side)
{
    for (int i=0 ; i < 5 ; i++)
    {
        Position pos = m_coordCalib->NormalizePosition(obstacles[i]->GetPos());
        if (m_roboObstaclePos[i].DistanceTo(pos) >= 0.01)
        {
            m_pathFinder.RemovePolygon(m_roboObstacles[i]);

            PathFinder::Point ul = PathFinder::CreatePoint(pos.GetX()-0.1, pos.GetY()-0.12);
            PathFinder::Point lr = PathFinder::CreatePoint(pos.GetX()+0.1, pos.GetY()+0.12);
            m_roboObstacles[i] = m_pathFinder.AddRectangle(ul, lr);
            m_roboObstaclePos[i] = pos;
        }
    }

    Position ballPos = m_coordCalib->NormalizePosition(m_ball->GetPos());
    if (ballPos.DistanceTo(m_ballObstaclePos) >= 0.01)
    {
        for (int i=0 ; i < 3 ; i++)
            m_pathFinder.RemovePolygon(m_ballObstacles[i]);

        double bx = ballPos.GetX(), by = ballPos.GetY();
        double goalX = our_side == LEFT_SIDE ? -1 : 1;
        double cosAngle, sinAngle;
        ComputeLineAngle(bx, by, goalX, 0, &cosAngle, &sinAngle);

        double x, y;
        ComputeVectorEnd(bx, by, cosAngle, sinAngle, 0.075, &x, &y);

        double x1, y1;
        ComputeVectorEnd(x, y, sinAngle, -cosAngle, 0.1, &x1, &y1);

        double x2, y2;
        ComputeVectorEnd(x, y, -sinAngle, cosAngle, 0.1, &x2, &y2);
        m_ballObstacles[0] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x1,y1), PathFinder::CreatePoint(x2,y2), 0.03);

        ComputeVectorEnd(x1, y1, -cosAngle, -sinAngle, 0.2, &x, &y);
        m_ballObstacles[1] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x1,y1), PathFinder::CreatePoint(x,y), 0.03);

        ComputeVectorEnd(x2, y2, -cosAngle, -sinAngle, 0.2, &x, &y);
        m_ballObstacles[2] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x2,y2), PathFinder::CreatePoint(x,y), 0.03);

        m_ballObstaclePos = ballPos;
    }
}

