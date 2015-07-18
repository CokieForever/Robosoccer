#include "teamrobot.h"
#include "matrix.h"
#include "goalkeeper.h"
#include "playertwo.h"
#include "refereedisplay.h"
#include "playermain.h"
#include "log.h"
#include "geometry.h"

/**
 * @brief Proof if path is valid
 *
 * @param path vector of calculated path
 * @param tgt Target position
 * @return bool returns false if no path is given
 */
bool TeamRobot::IsPathOK(PathFinder::Path path, PathFinder::Point& tgt)
{
    if (!path)
        return false;

    PathFinder::Point& last = path->back();
    return last.x == tgt.x && last.y == tgt.y;
}

/**
 * @brief Constructor of TeamRobot class
 *
 * @param DBC Database connexion
 * @param deviceNr Number of the robot
 * @param coordCalib Coordinates calibrator
 * @param ball Ball data
 * @param ballPm Monitoring the ball
 * @param display Display paths, robots ball
 */
TeamRobot::TeamRobot(RTDBConn& DBC, const int deviceNr, const CoordinatesCalibrer *coordCalib, RawBall *ball, BallMonitor *ballPm, RefereeDisplay *display) : NewRoboControl(DBC, deviceNr)
{
    m_coordCalib = coordCalib;
    m_ball = ball;
    m_display = display;
    m_pathFinderPath = NULL;
    m_map.Fill(0);
    m_prevFormation = Interpreter::INIT;     //Update request
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
    PathFinder::Point ul = PathFinder::CreatePoint(-2, -0.35);
    PathFinder::Point lr = PathFinder::CreatePoint(-0.8, 0.35);
    m_penaltyAreaObstacles[0] = m_pathFinder.AddRectangle(ul, lr);

    //Right penalty area
    ul = PathFinder::CreatePoint(0.8, -0.35);
    lr = PathFinder::CreatePoint(2, 0.35);
    m_penaltyAreaObstacles[1] = m_pathFinder.AddRectangle(ul, lr);

    AddBorderObstaclesToPathFinder();

    GiveDisplay(display);
    
    //collision detection starts
    //pthread_create(&m_thread, NULL, Checkspeed, this);
}

/**
 * @brief Add borders around obstacles, this is included in the path
 *
 * @param small
 */
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

/**
 * @brief TeamRobot destructor
 *
 */
TeamRobot::~TeamRobot()
{
    //m_checkSpeedFinishNow = true;
    //pthread_join(m_thread, NULL);
    if (m_pathFinderPath)
        delete m_pathFinderPath;
}

/**
 * @brief using this function the robot get the default position for the current situation
 *
 * @return Position is the default position.
 */
Position TeamRobot::getDefaultPosition() const
{
    return m_defaultPos;
}

/**
 * @brief this function defines the x of the default position
 *
 * @param x is the X value of current default position
 */
void TeamRobot::setDefaultPositionX(double x)
{
    m_defaultPos.SetX(x);
}

/**
 * @brief this function defines the x of the default position
 *
 * @param y is the Y value of current default position
 */
void TeamRobot::setDefaultPositionY(double y)
{
    m_defaultPos.SetY(y);
}

/**
 * @brief this function defines pos of the default position
 *
 * @param pos is the value of current default position
 */
void TeamRobot::setDefaultPosition(Position pos)
{
    m_defaultPos = pos;
}

/**
 * @brief
 *
 * @return const CoordinatesCalibrer *
 */
const CoordinatesCalibrer* TeamRobot::getCoordinatesCalibrer() const
{
    return m_coordCalib;
}

/**
 * @brief
 *
 * @return RawBall *
 */
RawBall* TeamRobot::getBall() const
{
    return m_ball;
}

/**
 * @brief this function calculate the actual position in the map based on the pathfinder.
 * it unnormalizes from pathfinder to real map
 * @param i is value on x axis in pathfinder
 * @param j is value on y axis in pathfinder
 * @return int
 */
int TeamRobot::getMapValue(int i, int j) const
{
    return (i<Interpreter::MAP_WIDTH && j<Interpreter::MAP_HEIGHT && i>0 && j>0) ? m_map[i][j] : 0;
}

/**
 * @brief
 *
 * @return const Interpreter::Map &
 */
const Interpreter::Map& TeamRobot::getMap() const
{
    return m_map;
}

/**
 * @brief
 *
 * @param i
 * @param j
 * @param val
 * @return bool
 */
bool TeamRobot::setMapValue(int i, int j, int val)
{
    if (i<Interpreter::MAP_WIDTH && j<Interpreter::MAP_HEIGHT && i>0 && j>0)
    {
        m_map[i][j] = val;
        return true;
    }
    return false;
}

/**
 * @brief
 *
 * @param map
 */
void TeamRobot::setMap(const Interpreter::Map &map)
{
    m_map = map;
}

/**
 * @brief
 *
 * @param display
 */
void TeamRobot::GiveDisplay(RefereeDisplay *display)
{
    m_display = display;
#if defined(PATHPLANNING_POLYGONS) && !defined(PATHPLANNING_ASTAR)
    if (m_display)
        m_display->DisplayPathFinder(&m_pathFinder);
#endif
}

/**
 * @brief update the path based on current position of the robots
 *
 * @param obstacles[] are all other robots
 * @param info is current playmode of referee
 */
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

/**
 * @brief Computation best path for a robot, take n points and interpolate, obstacles are taken into account. 
 * 
 * @param interpreter Get Interpreter informations
 */
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
            j = atoi(&c);
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

    endX = std::min(0.98, std::max(-0.98, endX));
    endY = std::min(0.98, std::max(-0.98, endY));
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

/**
 * @brief this function is carried out during the game and it allows the robot to attack and defense properly
 *
 * @param info is current playmode of referee
 */
void TeamRobot::FollowPath(const Interpreter::GameData& info)
{
    Position ballPos = m_ball->GetPos();

    bool kicked = true;
    if (true || info.formation == Interpreter::DEF)
    {
        if (ShouldGoalKick(ballPos, info.our_side))
            KickMovingBall(m_ball);
        else
            kicked = false;
    }
    else
    {
        Position goalPos(info.our_side == LEFT_SIDE ? 1 : -1, 0);
        goalPos = m_coordCalib->UnnormalizePosition(goalPos);
        if (ShouldKick(ballPos, goalPos))
            KickMovingBall(m_ball);
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


/**
 * @brief Perform kickoff, enemy robots are taken into account, after best way to shoot the ball is calculated, shoot it if our turn
 *
 * @param otherRobots[] Number all other robots
 * @param ourSide Get side information
 * @param likePenalty 
 */
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
    ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), dir, 0.01, &endX, &endY);

    if (likePenalty)
    {
        Position currentPos = m_coordCalib->NormalizePosition(GetPos());
        double cosAngle, sinAngle;
        double l = currentPos.DistanceTo(Position(endX, endY));
        ComputeLineAngle(currentPos.GetX(), currentPos.GetY(), endX, endY, &cosAngle, &sinAngle);
        ComputeVectorEnd(currentPos.GetX(), currentPos.GetY(), cosAngle, sinAngle, l+0.025, &endX, &endY);

        Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
        for (int j=0 ; GetPos().DistanceTo(end) >= 0.05 && j < 10 ; j++)
        {
            GotoXY(end.GetX(), end.GetY(), 50);
            for (int i=0 ; i < 20 && GetPos().DistanceTo(end) >= 0.05 ; i++)
                usleep(20000);
        }
    }
    else
    {
        Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
        while (!cruisetoBias(end.GetX(), end.GetY(), 400))
            usleep(1000);
    }

    if (likePenalty)
        KickBall(m_ball->GetPos());
    else
        KickMovingBall(m_ball);
}

/**
 * @brief Function for Penalty Shooting, find best path for robot to shoot the ball into a free area in the goal 
 *
 * @param otherRobots[] Number of other robot s
 */
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
    /*double dir;
    ComputeLineAngle(-1, 0, ballPos_n.GetX(), ballPos_n.GetY(), &dir);*/

    double endX, endY;
    dir = dir<=0 ? dir+M_PI : dir-M_PI;
    ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), dir, 0.075, &endX, &endY);

    Position end = m_coordCalib->UnnormalizePosition(Position(endX, endY));
    bool forward = true;

    for (int k=0 ; k < 3 ; k++)
    {
        if (k != 0 && GetPos().DistanceTo(end) > 0.02)
        {
            forward = Rotation(ballPos);
            for (int i=0 ; i < 50 ; i++)
            {
                if (forward)
                    MoveMs(-30, -30, 100, 0);
                else
                    MoveMs(30, 30, 100, 0);
                usleep(33000);
            }
        }
        else if (k != 0)
            break;

        for (int j=0 ; GetPos().DistanceTo(end) >= 0.05 && j < 3 ; j++)
        {
            GotoXY(end.GetX(), end.GetY(), 50);
            for (int i=0 ; i < 500 && GetPos().DistanceTo(end) >= 0.05 ; i++)
                usleep(20000);
        }
    }

    usleep(1e6);
    forward = Rotation(ballPos);
    usleep(1e6);
    KickBall(ballPos, false, forward);
}

bool TeamRobot::Rotation(Position ballPos)
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

    for (int i=0 ; i < 200 && fabs(diff3) >= 5 * M_PI / 180 ; i++)
    {

    #ifdef SIMULATION
        if (diff3 < 0)
            MoveMs(-20, 20, 100, 0);
        else if (diff3 > 0)
            MoveMs(20, -20, 100, 0);
    #else
        if (diff3 > 0)
            MoveMs(-20, 20, 100, 0);
        else if (diff3 < 0)
            MoveMs(20, -20, 100, 0);
    #endif

        usleep(20000);

        phi = GetPhi().Get();
        diff3 = AngleDiff(a, phi);

        Log("diff3 = " + ToString(diff3 * 180 / M_PI), DEBUG);
    }

    StopAction();
    return forward;
}

/**
 * @brief this function calculate whether the robot should move forwards or backwards to kick the ball. Furthermore, it adjusts the angle of the robot so that the robot can hit the ball precisely.
 *
 * @param ballPos is the position of the ball
 */
void TeamRobot::KickBall(Position ballPos, bool rotate, bool forward)
{
    if (rotate)
        forward = Rotation(ballPos);

    for (int i = 0 ; i < 10 ; i++)
    {
        if (forward)
            MoveMs(200, 200, 200, 0);
        else
            MoveMs(-200, -200, 200, 0);
        usleep(33000);
    }

    usleep(900000);
}

/**
 * @brief Function for kicking the ball while it is moving, robot drives behind the ball rotate to a good position and shoot
 *
 * @param ball Ball data
 */
void TeamRobot::KickMovingBall(RawBall *ball)
{
    double diff3 = PathFinder::INFINI_TY;
    bool forward;

    for (int i=0 ; i < 10 ; i++)
    {
        Position ballPos = ball->GetPos();
        if (ballPos.DistanceTo(GetPos()) > 0.2)
            return;

        double phi = GetPhi().Get();

        double robotBallAngle;
        Position pos = GetPos();
        ComputeLineAngle(pos.GetX(), pos.GetY(), ballPos.GetX(), ballPos.GetY(), &robotBallAngle);

        double diff1 = AngleDiff(robotBallAngle, phi);

        double robotBallAngle2 = robotBallAngle<=0 ? robotBallAngle+M_PI : robotBallAngle-M_PI;
        double diff2 = AngleDiff(robotBallAngle2, phi);

        forward = fabs(diff1) < fabs(diff2);
        double a = forward ? robotBallAngle : robotBallAngle2;
        diff3 = AngleDiff(a, phi);

        Log("diff3 = " + ToString(diff3 * 180 / M_PI), DEBUG);
        if (abs(diff3) <= 5 * M_PI / 180)
            break;

    #ifdef SIMULATION
        double time = std::min(40., fabs(diff3) * 2710 / (3 * M_PI));
        if (diff3 < 0)
            MoveMs(-60, 60, 200, 0);
        else if (diff3 > 0)
            MoveMs(60, -60, 200, 0);
    #else
        double time = std::min(40., fabs(diff3) * 1355 / (3 * M_PI));
        if (diff3 > 0)
            MoveMs(-60, 60, time, 0);
        else if (diff3 < 0)
            MoveMs(60, -60, time, 0);
    #endif

        usleep(time * 1000);
        StopAction();
    }

    StopAction();

    for (int i = 0 ; i < 10 ; i++)
    {
        if (forward)
            MoveMs(200, 200, 600, 0);
        else
            MoveMs(-200, -200, 600, 0);
        usleep(33000);
    }
}

/**
 * @brief this function calculate if the robot should kick the ball and kick the ball
 * in the direction of the goal.
 * @param ballPos is the position of the ball
 * @param goalPos is the position of the goal
 * @return bool is false if the ball is still far away from the robot
 */
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

/**
 * @brief this function let the robot kick the ball if the ball is on our side.
 * in this way the potential dangerous is eliminated
 *
 * @param ballPos is the position of the ball
 * @param ourSide the side we are right now.
 * @return bool return false if ball is still far away from the robot
 */
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

/**
 * @brief this function calculate the difference between 2 angles
 * and return the difference value in the range of -PI to PI.
 * @param angle1
 * @param angle2
 * @return double the difference between angle1 and angle2
 */
double TeamRobot::AngleDiff(double angle1, double angle2)
{
    double diff = fmod(angle1 - angle2 + 2*M_PI, 2*M_PI);
    if (diff >= M_PI)
        diff -= 2*M_PI;
    return diff;
}


/**
 * @brief moves the robot to a random position if it stucks somewhere.
 *
 * @param data is the robot
 */
void* TeamRobot::Checkspeed(void *data)
{
    TeamRobot *robo = (TeamRobot*)data;
    robo->m_checkSpeedFinishNow = false;

    Position current;
    while(!robo->m_checkSpeedFinishNow)
    {
        current = robo->GetPos();
        usleep(5000000);
        Log("Collision detection.",INFO);

        if(!robo->IsOnTarget(robo->m_targetPos))
        //if(((abs(robo->m_targetPos.GetX()-current.GetX())+abs(robo->m_targetPos.GetY()-current.GetY()))>0.2) && (robo->m_targetSpeed!=0))
        {
            robo->RandomMove();
            Log("Robot is random moving.", INFO);
        }
    }
    return 0;
}
