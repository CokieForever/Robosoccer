#include "teamrobot.h"
#include "matrix.h"
#include "goalkeeper.h"
#include "playertwo.h"
#include "refereedisplay.h"
#include "playermain.h"
#include "log.h"
#include "geometry.h"

bool TeamRobot::IsPathOK(PathFinder::Path path, PathFinder::Point& tgt)
{
    if (!path)
        return false;

    PathFinder::Point& last = path->back();
    return last.x == tgt.x && last.y == tgt.y;
}

TeamRobot::TeamRobot(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *ball, BallMonitor *ballPm, RefereeDisplay *display) : NewRoboControl(DBC, deviceNr)
{
    m_coordCalib = coordCalib;
    m_ball = ball;
    m_display = display;
    m_pathFinderPath = NULL;
    m_map.Fill(0);
    m_prevFormation = Interpreter::MIX;     //Update request
    m_areaObstacle = NULL;
    m_ballPm = ballPm;

    m_ballObstaclePos = Position(-10, -10);
    for (int i=0 ; i < 3 ; i++)
        m_ballObstacles[i] = NULL;
    for (int i=0 ; i < 5 ; i++)
    {
        m_roboObstacles[i] = NULL;
        m_roboObstaclePos[i] = Position(-10, -10);
    }

    //Left penalty area
    PathFinder::Point ul = PathFinder::CreatePoint(-2, -0.3);
    PathFinder::Point lr = PathFinder::CreatePoint(-0.8, 0.3);
    m_penaltyAreaObstacles[0] = m_pathFinder.AddRectangle(ul, lr);

    //Right penalty area
    ul = PathFinder::CreatePoint(0.8, -0.3);
    lr = PathFinder::CreatePoint(2, 0.3);
    m_penaltyAreaObstacles[1] = m_pathFinder.AddRectangle(ul, lr);

    AddBorderObstaclesToPathFinder();

    GiveDisplay(display);
}

void TeamRobot::AddBorderObstaclesToPathFinder(bool small)
{
    //Upper border
    PathFinder::Point ul = PathFinder::CreatePoint(-1, -2);
    PathFinder::Point lr = PathFinder::CreatePoint(1, small ? -1 : -0.95);
    m_borderObstacles[0] = m_pathFinder.AddRectangle(ul, lr);

    //Lower border
    ul = PathFinder::CreatePoint(-1, small ? 1 : 0.95);
    lr = PathFinder::CreatePoint(1, 2);
    m_borderObstacles[1] = m_pathFinder.AddRectangle(ul, lr);

    //Left border
    ul = PathFinder::CreatePoint(-2, -1);
    lr = PathFinder::CreatePoint(small ? -1 : -0.95, 1);
    m_borderObstacles[2] = m_pathFinder.AddRectangle(ul, lr);

    //Right border
    ul = PathFinder::CreatePoint(small ? 1 : 0.95, -1);
    lr = PathFinder::CreatePoint(2, 1);
    m_borderObstacles[3] = m_pathFinder.AddRectangle(ul, lr);
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

const CoordinatesCalibrer* TeamRobot::getCoordinatesCalibrer() const
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
    m_map = map;
}

void TeamRobot::GiveDisplay(RefereeDisplay *display)
{
    m_display = display;
#if defined(PATHPLANNING_POLYGONS) && !defined(PATHPLANNING_ASTAR)
    if (m_display)
        m_display->DisplayPathFinder(&m_pathFinder);
#endif
}

void TeamRobot::UpdatePathFinder(const NewRoboControl* obstacles[5], const Interpreter::GameData& info)
{
    //Add obstacles around robots
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

    //Add obstacles around ball
    Position ballPos = m_coordCalib->NormalizePosition(m_ball->GetPos());
    if (ballPos.DistanceTo(m_ballObstaclePos) >= 0.01)
    {
        m_pathFinder.RemovePolygons(m_ballObstacles, 3);

        double bx = ballPos.GetX(), by = ballPos.GetY();
        double goalX = info.our_side == LEFT_SIDE ? 1 : -1;
        double cosAngle, sinAngle;
        ComputeLineAngle(bx, by, goalX, 0, &cosAngle, &sinAngle);

        double x, y;
        ComputeVectorEnd(bx, by, cosAngle, sinAngle, 0.085, &x, &y);

        double x1, y1;
        ComputeVectorEnd(x, y, sinAngle, -cosAngle, 0.1, &x1, &y1);

        double x2, y2;
        ComputeVectorEnd(x, y, -sinAngle, cosAngle, 0.1, &x2, &y2);
        m_ballObstacles[0] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x1,y1), PathFinder::CreatePoint(x2,y2), 0.1);

        ComputeVectorEnd(x1, y1, -cosAngle, -sinAngle, 0.15, &x, &y);
        m_ballObstacles[1] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x1,y1), PathFinder::CreatePoint(x,y), 0.05);

        ComputeVectorEnd(x2, y2, -cosAngle, -sinAngle, 0.15, &x, &y);
        m_ballObstacles[2] = m_pathFinder.AddThickLine(PathFinder::CreatePoint(x2,y2), PathFinder::CreatePoint(x,y), 0.05);

        m_ballObstaclePos = ballPos;
    }

    //Limit the accessible area
    if (info.formation != m_prevFormation)
    {
        m_pathFinder.RemovePolygon(m_areaObstacle);
        AddObstacleForFormation(info);
        m_prevFormation = info.formation;
    }
}

void TeamRobot::ComputePath(const Interpreter& interpreter)
{
#ifdef PATHPLANNING_ASTAR

    //both needed when it comes to path tracking
    Interpreter::Point A,B;
    //Interpreter::Point *pt;
    char c;
    int j,idx_tmp;
    unsigned int interpolate_n = 15;
    Position robo_n,ball_n;

    interpreter.SetP1MapToRobot(this);
    if (m_q.size()==0)
    {
        //create queue of move indices

        robo_n = m_coordCalib->NormalizePosition(GetPos());
        ball_n = m_coordCalib->NormalizePosition(m_ball->GetPos());

        A.x = Interpreter::coord2mapX(robo_n.GetX());
        A.y = Interpreter::coord2mapY(robo_n.GetY());
        B.x = Interpreter::coord2mapX(ball_n.GetX());
        B.y = Interpreter::coord2mapY(ball_n.GetY());

        //get string with motion commands
        m_path = Interpreter::pathFind(m_map,A,B);
        #ifdef VERY_VERBOSE
        Interpreter::showMap(m_map,m_path,A);
        #endif

        for (unsigned int i= 0 ; i<m_path.length() ; i++)
        {
            c=m_path.at(i);
            j=atoi(&c);
            m_q.push(j);
        }
    }
    m_go_x = 0;
    m_go_y = 0;

    //take n points and interpolate
    if (m_q.size()>= interpolate_n)
    {
        for (unsigned int i=0;i<interpolate_n;i++)
        {
            idx_tmp = m_q.front();
            m_go_x = m_go_x + Interpreter::DX[idx_tmp];
            m_go_y = m_go_y + Interpreter::DY[idx_tmp];
            m_q.pop();
        }

    }
    else
    {
        while(m_q.size()!=0)
        {
            idx_tmp = m_q.front();
            m_go_x = m_go_x + Interpreter::DX[idx_tmp];
            m_go_y = m_go_y + Interpreter::DY[idx_tmp];
            m_q.pop();
        }
    }

#elif defined(PATHPLANNING_POLYGONS)

    if (m_pathFinderPath)
        delete m_pathFinderPath;

    //TODO Dirty - Implement child method for this
    const NewRoboControl *p1 = interpreter.getP1();
    const NewRoboControl *p2 = interpreter.getP2();
    const NewRoboControl *gk = interpreter.getGK();
    const NewRoboControl* robots[5] = {gk, p2, interpreter.getE1(), interpreter.getE2(), interpreter.getE3()};
    if (p2 == (NewRoboControl*)this)
        robots[1] = p1;
    else if (gk == (NewRoboControl*)this)
    {
        robots[0] = p1;
        robots[1] = p2;
    }

    UpdatePathFinder(robots, interpreter.getMode());

    Position robo_n = m_coordCalib->NormalizePosition(GetPos());
    Position ball_n = m_coordCalib->NormalizePosition(m_ball->GetPos());
    PathFinder::Point start = PathFinder::CreatePoint(robo_n.GetX(), robo_n.GetY());

    eSide side = interpreter.getMode().our_side;
    Position goalPos(side == LEFT_SIDE ? 1 : -1, 0);
    double endX, endY, cosAngle, sinAngle;
    ComputeLineAngle(goalPos.GetX(), goalPos.GetY(), ball_n.GetX(), ball_n.GetY(), &cosAngle, &sinAngle);
    ComputeVectorEnd(ball_n.GetX(), ball_n.GetY(), cosAngle, sinAngle, 0.1, &endX, &endY);

    if (fabs(endX) >= 1 || fabs(endY) >= 1)
    {
        double isectX[2], isectY[2];
        if (GetSegmentRectIntersections(-0.98, -0.98, 0.98, 0.98, ball_n.GetX(), ball_n.GetY(), endX, endY, isectX, isectY) > 0)
        {
            endX = isectX[0];
            endY = isectY[0];
        }
        else
        {
            endX = ball_n.GetX();
            endY = ball_n.GetY();
        }
    }

    PathFinder::Point end = PathFinder::CreatePoint(endX, endY);

    m_pathFinderPath = m_pathFinder.ComputePath(start, end);
    if (!IsPathOK(m_pathFinderPath, end))
    {
        if (m_pathFinderPath)
            delete m_pathFinderPath;
        m_pathFinder.RemovePolygons(m_borderObstacles, 4);
        AddBorderObstaclesToPathFinder(true);
        m_pathFinderPath = m_pathFinder.ComputePath(start, end);
        if (!IsPathOK(m_pathFinderPath, end))
        {
            if (m_pathFinderPath)
                delete m_pathFinderPath;
            m_pathFinder.RemovePolygon(m_ballObstacles[1]);
            m_pathFinder.RemovePolygon(m_ballObstacles[2]);
            m_ballObstacles[1] = NULL;
            m_ballObstacles[2] = NULL;
            m_pathFinderPath = m_pathFinder.ComputePath(start, end);
            if (!m_pathFinderPath)
            {
                PathFinder::Point pt = m_pathFinder.ComputeClosestAccessiblePoint(start, end);
                if (pt.x != start.x || pt.y != start.y)
                    m_pathFinderPath = m_pathFinder.ComputePath(start, pt);
            }
            m_ballObstaclePos = Position(-10, -10); //Update request
        }
        m_pathFinder.RemovePolygons(m_borderObstacles, 4);
        AddBorderObstaclesToPathFinder(false);
    }

    if (m_display)
        m_display->DisplayPath(m_pathFinderPath);

#endif
}

void TeamRobot::FollowPath(const Interpreter::GameData& info)
{
    Position ballPos;
    m_ballPm->GetBallPosition(&ballPos);

    bool kicked = true;
    if (info.formation == Interpreter::DEF)
    {
        if (ShouldGoalKick(ballPos, info.our_side))
            KickBall(ballPos);
        else
            kicked = false;
    }
    else
    {
        Position goalPos(info.our_side == LEFT_SIDE ? 1 : -1, 0);
        goalPos = m_coordCalib->UnnormalizePosition(goalPos);
        if (ShouldKick(ballPos, goalPos))
            KickBall(ballPos);
        else
            kicked = false;
    }

    if (!kicked)
    {
    #ifdef PATHPLANNING_ASTAR

        Log("Player1 Perform Followpath (queue size): " + ToString(m_q.size()), DEBUG);

        Position pos = m_coordCalib->NormalizePosition(GetPos());
        int mapx = Interpreter::coord2mapX(pos.GetX())+ m_go_x;
        int mapy = Interpreter::coord2mapY(pos.GetY())+ m_go_y;

        pos.SetX(Interpreter::map2coordX(mapx));
        pos.SetY(Interpreter::map2coordY(mapy));
        pos = m_coordCalib->UnnormalizePosition(pos.GetPos());

        cruisetoBias(pos.GetX(), pos.GetY(), 600, -10, 30);

    #elif defined(PATHPLANNING_POLYGONS)

        if (m_pathFinderPath)
        {
            std::vector<Position>* posList = PathFinder::ConvertPathToReal(m_pathFinderPath, m_coordCalib);
            drivePath(posList);
            delete posList;
        }
        else
            RandomMove();

    #endif
    }
}


void TeamRobot::KickOff(const NewRoboControl* otherRobots[5], eSide ourSide, bool likePenalty)
{
    NewRoboControl *robots[6];
    memcpy(robots, otherRobots, sizeof(NewRoboControl*)*5);
    robots[5] = this;

    std::vector<double> map = m_ballPm->ComputeVisibilityMap(1, (const NewRoboControl**)robots, ourSide, m_coordCalib);
    double dir = BallMonitor::GetBestDirection(map, ourSide);

    Position ballPos;
    m_ballPm->GetBallPosition(&ballPos);
    Position ballPos_n = m_coordCalib->NormalizePosition(ballPos);

    double endX, endY;
    dir = dir<=0 ? dir+M_PI : dir-M_PI;
    ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), dir, 0.075, &endX, &endY);

    if (likePenalty)
    {
        Position currentPos = m_coordCalib->NormalizePosition(GetPos());
        double cosAngle, sinAngle;
        double l = currentPos.DistanceTo(Position(endX, endY));
        ComputeLineAngle(currentPos.GetX(), currentPos.GetY(), endX, endY, &cosAngle, &sinAngle);
        ComputeVectorEnd(currentPos.GetX(), currentPos.GetY(), cosAngle, sinAngle, l+0.05, &endX, &endY);

        Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
        for (int j=0 ; GetPos().DistanceTo(end) >= 0.05 && j < 10 ; j++)
        {
            GotoXY(end.GetX(), end.GetY(), 50);
            for (int i=0 ; i < 50 && GetPos().DistanceTo(end) >= 0.05 ; i++)
                usleep(20000);
        }
    }
    else
    {
        Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
        while (!cruisetoBias(end.GetX(), end.GetY(), 400))
            usleep(1000);
    }

    KickBall(ballPos, likePenalty);
}

void TeamRobot::KickPenalty(const NewRoboControl* otherRobots[5])
{
    Position ballPos;
    m_ballPm->GetBallPosition(&ballPos);
    Position ballPos_n = m_coordCalib->NormalizePosition(ballPos);

    Position ePos[3];
    ePos[0] = m_coordCalib->NormalizePosition(otherRobots[2]->GetPos());
    ePos[1] = m_coordCalib->NormalizePosition(otherRobots[3]->GetPos());
    ePos[2] = m_coordCalib->NormalizePosition(otherRobots[4]->GetPos());

    std::vector<double> map = m_ballPm->ComputeVisibilityMap(1, ballPos, &(ePos[0]), 3, RIGHT_SIDE);
    double dir = BallMonitor::GetBestDirection(map, RIGHT_SIDE);

    double endX, endY;
    dir = dir<=0 ? dir+M_PI : dir-M_PI;
    ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), dir, 0.1, &endX, &endY);

    Position currentPos = m_coordCalib->NormalizePosition(GetPos());
    double cosAngle, sinAngle;
    double l = currentPos.DistanceTo(Position(endX, endY));
    ComputeLineAngle(currentPos.GetX(), currentPos.GetY(), endX, endY, &cosAngle, &sinAngle);
    ComputeVectorEnd(currentPos.GetX(), currentPos.GetY(), cosAngle, sinAngle, l+0.05, &endX, &endY);

    Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
    for (int j=0 ; GetPos().DistanceTo(end) >= 0.05 && j < 10 ; j++)
    {
        GotoXY(end.GetX(), end.GetY(), 50);
        for (int i=0 ; i < 50 && GetPos().DistanceTo(end) >= 0.05 ; i++)
            usleep(20000);
    }

    KickBall(ballPos, true);
}

void TeamRobot::KickBall(Position ballPos, bool precise)
{
    double phi = GetPhi().Get();

    double robotBallAngle;
    Position pos = GetPos();
    ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);

    double diff1 = AngleDiff(robotBallAngle, phi);

    double robotBallAngle2 = robotBallAngle<=0 ? robotBallAngle+M_PI : robotBallAngle-M_PI;
    double diff2 = AngleDiff(robotBallAngle2, phi);

    bool forward = fabs(diff1) < fabs(diff2);
    double a = forward ? robotBallAngle : robotBallAngle2;
    double diff3 = AngleDiff(a, phi);

    if (!precise)
    {
        double time = fabs(diff3) * 2710 / (2 * M_PI);
        if (diff3 > 0)
            MoveMs(-30, 30, 200, 0);
        else if (diff3 < 0)
            MoveMs(30, -30, 200, 0);

        usleep(time);
    }
    else
    {
        for (int i=0 ; i < 200 && fabs(diff3) >= 5 * M_PI / 180 ; i++)
        {
            if (diff3 > 0)
                MoveMs(-20, 20, 200, 0);
            else if (diff3 < 0)
                MoveMs(20, -20, 200, 0);

            usleep(20000);

            phi = GetPhi().Get();
            diff3 = AngleDiff(a, phi);
        }
    }

    MoveMs(0,0,0,0);

    if (forward)
        MoveMsBlocking(200, 200, 500, 0);
    else
        MoveMsBlocking(-200, -200, 500, 0);
}

bool TeamRobot::ShouldKick(Position ballPos, Position goalPos)
{
    Position pos = GetPos();
    if (pos.DistanceTo(ballPos) <= 0.2)
    {
        double robotGoalAngle, robotBallAngle;

        ComputeLineAngle(pos.GetX(), pos.GetY(), goalPos.GetX(), goalPos.GetY(), &robotGoalAngle);
        ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);
        double diff1 = AngleDiff(robotBallAngle, robotGoalAngle );

        return fabs(diff1) <= 10 * M_PI / 180;
    }

    return false;
}

bool TeamRobot::ShouldGoalKick(Position ballPos, eSide ourSide)
{
    double robotBallAngle;
    Position pos = GetPos();

    if (pos.DistanceTo(ballPos) <= 0.2)
    {
        ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);
        if (ourSide == RIGHT_SIDE)
            robotBallAngle = robotBallAngle<=0 ? robotBallAngle+M_PI : robotBallAngle-M_PI;

        return fabs(robotBallAngle) <= 45 * M_PI / 180;
    }

    return false;
}

double TeamRobot::AngleDiff(double angle1, double angle2)
{
    double diff = fmod(angle1 - angle2 + 2*M_PI, 2*M_PI);
    if (diff >= M_PI)
        diff -= 2*M_PI;
    return diff;
}
