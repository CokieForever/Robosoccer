#include "ballmonitor.h"
#include "pathfinder.h"
#include <queue>
#include "geometry.h"

BallMonitor::BallMonitor(CoordinatesCalibrer *coordCalibrer, RawBall *ball)
{
    m_mainBall = ball;
    m_stopBallMonitoring = false;
    m_ballMonitoring = false;
    m_ballMonitoringThread = NULL;
    m_ballFollowing = false;
    m_ballFollowingThread = NULL;
    m_stopBallFollowing = false;
    m_ballPosTimeInd = 0;
    m_nbBallPosTime = 0;
    m_followerRobot = NULL;
    m_coordCalibrer = coordCalibrer;
}

bool BallMonitor::StartMonitoring(RawBall *ball)
{
    if (m_ballMonitoring || !m_coordCalibrer)
        return false;
    else
    {
        if (ball)
            m_mainBall = ball;
        if (!m_mainBall)
            return false;

        pthread_mutex_init(&m_ballMonitoringMtx, NULL);
        pthread_create(&m_ballMonitoringThread, NULL, BallMonitoringFn, this);
        usleep(0.1e6);
        return true;
    }
}

bool BallMonitor::StopMonitoring()
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        m_stopBallMonitoring = true;
        pthread_join(m_ballMonitoringThread, NULL);
        pthread_mutex_destroy(&m_ballMonitoringMtx);
        return true;
    }
}

bool BallMonitor::GetBallPosition(Position *pos) const
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock((pthread_mutex_t*)&m_ballMonitoringMtx);
        if (m_nbBallPosTime < 1)
        {
            pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);
            return false;
        }
        *pos = m_ballPosTime[m_ballPosTimeInd].pos;
        pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);
        return true;
    }
}

bool BallMonitor::GetBallDirection(Direction *dir) const
{
    if (!m_ballMonitoring)
        return false;
    else
    {
        pthread_mutex_lock((pthread_mutex_t*)&m_ballMonitoringMtx);
        if (m_nbBallPosTime < 2)
        {
            pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);
            return false;
        }

        int i2 = m_ballPosTimeInd;
        int i1 = (m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
        double t = (m_ballPosTime[i2].time - m_ballPosTime[i1].time) / (double)CLOCKS_PER_SEC;
        dir->x = (m_ballPosTime[i2].pos.GetX() - m_ballPosTime[i1].pos.GetX()) / t;
        dir->y = (m_ballPosTime[i2].pos.GetY() - m_ballPosTime[i1].pos.GetY()) / t;
        pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);

        return true;
    }
}

bool BallMonitor::PredictBallPosition(double *a, double *b, int precision)
{
    if (!m_ballMonitoring)
        return false;
    else if (!IsBallMoving())
    {
        ResetPosTimeList();
        return false;
    }
    else if (precision > 1 && m_nbBallPosTime < std::min(NB_POSTIME, precision))
        return false;
    else
    {
        Position ballPos1, ballPos2;

        pthread_mutex_lock(&m_ballMonitoringMtx);
        ballPos1 = m_coordCalibrer->NormalizePosition(m_ballPosTime[(m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME].pos);
        ballPos2 = m_coordCalibrer->NormalizePosition(m_ballPosTime[m_ballPosTimeInd].pos);
        pthread_mutex_unlock(&m_ballMonitoringMtx);

        if ((precision > 1 && !ComputeLinearRegression(a, b, precision)) || (precision <= 1 && fabs(ballPos2.GetX() - ballPos1.GetX()) <= 1e-4))
        {
            *a = PathFinder::INFINI_TY;
            *b = PathFinder::INFINI_TY;
            return true;
        }
        else if (precision <= 1)
        {
            *a = (ballPos2.GetY() - ballPos1.GetY()) / (ballPos2.GetX() - ballPos1.GetX());
            *b = ballPos2.GetY() - (*a) * ballPos2.GetX();
        }

        return true;
    }
}

bool BallMonitor::IsBallMoving() const
{
    pthread_mutex_lock((pthread_mutex_t*)&m_ballMonitoringMtx);
    if (m_nbBallPosTime < 2)
    {
        pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);
        return false;
    }
    int i2 = m_ballPosTimeInd;
    int i1 = (m_ballPosTimeInd-1 + NB_POSTIME) % NB_POSTIME;
    bool moving = (m_ballPosTime[i1].pos.GetX() != m_ballPosTime[i2].pos.GetX()
            || m_ballPosTime[i1].pos.GetY() != m_ballPosTime[i2].pos.GetY())
            && clock() - m_ballPosTime[i1].time <= CLOCKS_PER_SEC * 0.100;
    pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);
    return moving;
}

double BallMonitor::GetBestDirection(std::vector<double> visibilityMap, eSide ourSide)
{
    int n = visibilityMap.size();
    if (n > 0)
    {
        double angle = 0, maxAngle = -1;
        double prevEnd = -10, prevStart = 0;
        for (int i=0 ; i < n ; i += 2)
        {
            double start = visibilityMap[i];
            double end = visibilityMap[i+1];
            if (start == prevEnd)
                start = prevStart;
            else
                prevStart = start;

            if (end - start > maxAngle)
            {
                maxAngle = end - start;
                angle = (end + start) / 2;
            }

            prevEnd = end;
        }

        if (prevEnd == M_PI && visibilityMap[0] == -M_PI)
        {
            prevEnd = -M_PI;
            prevStart -= 2*M_PI;
            for (int i=0 ; prevEnd == visibilityMap[i] ; i += 2)
            {
                double end = visibilityMap[i+1];
                if (end - prevStart > maxAngle)
                {
                    maxAngle = end - prevStart;
                    angle = (end + prevStart) / 2;
                }

                prevEnd = end;
            }

            if (angle < -M_PI)
                angle += 2*M_PI;
        }

        return angle;
    }
    else
        return ourSide == LEFT_SIDE ? 0 : M_PI;
}

std::vector<double> BallMonitor::ComputeVisibilityMap(int maxLevel, const NewRoboControl* robot[6], eSide ourSide, const CoordinatesCalibrer *coordCalib) const
{
    Position robotPos[6];
    for (int i=0 ; i < 6 ; i++)
        robotPos[i] = coordCalib->NormalizePosition(robot[i]->GetPos());

    Position ballPos;
    GetBallPosition(&ballPos);
    ballPos = coordCalib->NormalizePosition(ballPos);
    return ComputeVisibilityMap(maxLevel, ballPos, &(robotPos[0]), 6, ourSide);
}

std::vector<double> BallMonitor::ComputeVisibilityMap(int maxLevel, Position pos, const Position *robotPos, int nbPos, eSide ourSide)
{
    double minAngle, maxAngle;
    double goalPosX = ourSide == LEFT_SIDE ? 1 : -1;
    ComputeLineAngle(pos.GetX(), pos.GetY(), goalPosX, -0.2, &minAngle);
    ComputeLineAngle(pos.GetX(), pos.GetY(), goalPosX, +0.2, &maxAngle);
    if (pos.GetX() < goalPosX)
        std::swap(minAngle, maxAngle);

    int bx = floor((fabs(pos.GetX())+1) / 2);
    int by = floor((fabs(pos.GetY())+1) / 2);
    if (pos.GetX() < -1)
        bx = -bx;
    if (pos.GetY() < -1)
        by = -by;

    std::vector<double> baseMap = ComputeVisibilityMap(pos, robotPos, nbPos, ourSide);

    if (maxLevel <= 0)
        return baseMap;

    double xMin = 2*fabs(bx) - 1, xMax = xMin + 2;
    double yMin = 2*fabs(by) - 1, yMax = yMin + 2;
    if (pos.GetX() < -1)
    {
        double x = xMin;
        xMin = -xMax;
        xMax = -x;
    }
    if (pos.GetY() < -1)
    {
        double y = yMin;
        yMin = -yMax;
        yMax = -y;
    }

    Position pos2;
    Position *robotPos2 = new Position[nbPos*2];
    memcpy(robotPos2, robotPos, sizeof(Position)*nbPos);
    std::vector<double> map;
    int n;

    //Rebounce on North side
    pos2 = Position(pos.GetX(), 2*yMin-pos.GetY());
    for (int i=0 ; i < nbPos ; i++)
        robotPos2[i+nbPos] = Position(robotPos[i].GetX(), 2*yMin-robotPos[i].GetY());

    map = ComputeVisibilityMap(maxLevel-1, pos2, robotPos2, nbPos*2, ourSide);
    n = map.size();
    for (int i=0 ; i < n ; i++)
        map[i] = -map[i];
    std::reverse(map.begin(), map.end());
    baseMap = MergeVisibilityMaps(baseMap, map);

    //Rebounce on South side
    pos2 = Position(pos.GetX(), 2*yMax-pos.GetY());
    for (int i=0 ; i < nbPos ; i++)
        robotPos2[i+nbPos] = Position(robotPos[i].GetX(), 2*yMax-robotPos[i].GetY());

    map = ComputeVisibilityMap(maxLevel-1, pos2, robotPos2, nbPos*2, ourSide);
    n = map.size();
    for (int i=0 ; i < n ; i++)
        map[i] = -map[i];
    std::reverse(map.begin(), map.end());
    baseMap = MergeVisibilityMaps(baseMap, map);

    //Rebounce on East side
    pos2 = Position(2*xMax-pos.GetX(), pos.GetY());
    for (int i=0 ; i < nbPos ; i++)
        robotPos2[i+nbPos] = Position(2*xMax-robotPos[i].GetX(), robotPos[i].GetY());

    map = ComputeVisibilityMap(maxLevel-1, pos2, robotPos2, nbPos*2, ourSide);
    n = map.size();
    std::vector<double> part1, part2;
    int k;
    for (k=0 ; k < n && map[k] < 0 ; k++)
        part1.push_back(AngleVertMirror(map[k]));
    if (k % 2 == 1)
    {
        part1.push_back(-M_PI);
        part2.push_back(M_PI);
    }
    std::reverse(part1.begin(), part1.end());
    for (; k < n ; k++)
        part2.push_back(AngleVertMirror(map[k]));
    std::reverse(part2.begin(), part2.end());
    map = part1;
    map.insert(map.end(), part2.begin(), part2.end());
    baseMap = MergeVisibilityMaps(baseMap, map);

    //Rebounce on West side
    pos2 = Position(2*xMin-pos.GetX(), pos.GetY());
    for (int i=0 ; i < nbPos ; i++)
        robotPos2[i+nbPos] = Position(2*xMin-robotPos[i].GetX(), robotPos[i].GetY());

    map = ComputeVisibilityMap(maxLevel-1, pos2, robotPos2, nbPos*2, ourSide);
    n = map.size();
    part1.clear(); part2.clear();
    for (k=0 ; k < n && map[k] < 0 ; k++)
        part1.push_back(AngleVertMirror(map[k]));
    if (k % 2 == 1)
    {
        part1.push_back(-M_PI);
        part2.push_back(M_PI);
    }
    std::reverse(part1.begin(), part1.end());
    for (; k < n ; k++)
        part2.push_back(AngleVertMirror(map[k]));
    std::reverse(part2.begin(), part2.end());
    map = part1;
    map.insert(map.end(), part2.begin(), part2.end());
    baseMap = MergeVisibilityMaps(baseMap, map);

    delete robotPos2;
    return baseMap;
}

std::vector<double> BallMonitor::ComputeVisibilityMap(const NewRoboControl* robot[6], eSide ourSide, const CoordinatesCalibrer *coordCalib) const
{
    Position robotPos[6];
    for (int i=0 ; i < 6 ; i++)
        robotPos[i] = coordCalib->NormalizePosition(robot[i]->GetPos());

    Position ballPos;
    GetBallPosition(&ballPos);
    ballPos = coordCalib->NormalizePosition(ballPos);
    return ComputeVisibilityMap(ballPos, &(robotPos[0]), 6, ourSide);
}

std::vector<double> BallMonitor::ComputeVisibilityMap(Position pos, const Position *robotPos, int nbPos, eSide ourSide)
{
    double minAngle, maxAngle;
    double goalPosX = ourSide == LEFT_SIDE ? 1 : -1;
    ComputeLineAngle(pos.GetX(), pos.GetY(), goalPosX, -0.2, &minAngle);
    ComputeLineAngle(pos.GetX(), pos.GetY(), goalPosX, +0.2, &maxAngle);
    if (pos.GetX() > goalPosX)
        std::swap(minAngle, maxAngle);

    const double diameter = 0.075;
    priority_queue<Angle, std::vector<Angle>, CompareFn> angles(CompareAngles);

    int id = 0;
    for (int i=0 ; i < nbPos ; i++)
    {
        double d1 = pos.DistanceTo(robotPos[i]);
        double d2 = sqrt(d1*d1 + diameter*diameter);

        double angle;
        ComputeLineAngle(pos.GetX(), pos.GetY(), robotPos[i].GetX(), robotPos[i].GetY(), &angle);

        double c = acos(d1 / d2);

        if (angle - c < -M_PI)
        {
            angles.push(CreateAngle(-M_PI, id));
            angles.push(CreateAngle(M_PI, id));
            angles.push(CreateAngle(angle-c + 2*M_PI, id));
        }
        else
            angles.push(CreateAngle(angle-c, id));

        if (angle + c > M_PI)
        {
            angles.push(CreateAngle(M_PI, id));
            angles.push(CreateAngle(-M_PI, id));
            angles.push(CreateAngle(angle+c - 2*M_PI, id));
        }
        else
            angles.push(CreateAngle(angle+c, id));

        id++;
    }

    if (minAngle <= maxAngle)
        return AnglesToMap(angles, minAngle, maxAngle);
    else
    {
        std::vector<double> map1 = AnglesToMap(angles, minAngle, M_PI);
        std::vector<double> map2 = AnglesToMap(angles, -M_PI, maxAngle);
        return MergeVisibilityMaps(map1, map2);
    }
}

std::vector<double> BallMonitor::AnglesToMap(priority_queue<Angle, std::vector<Angle>, CompareFn> angles, double minAngle, double maxAngle)
{
    std::vector<int> currentIds;
    std::vector<double> map;
    map.push_back(minAngle);

    while (!angles.empty())
    {
        Angle angle = angles.top();
        angles.pop();

        double val = std::min(maxAngle, std::max(minAngle, angle.val));

        vector<int>::iterator it = find(currentIds.begin(), currentIds.end(), angle.id);
        if (it != currentIds.end())
        {
            currentIds.erase(it);
            if (currentIds.empty())
                map.push_back(val);
        }
        else
        {
            if (currentIds.empty())
                map.push_back(val);
            currentIds.push_back(angle.id);
        }
    }

    map.push_back(maxAngle);

    //Remove doubles
    for (int i = map.size()-1 ; i >= 0 ; i -= 2)
    {
        if (map[i] == map[i-1])
            map.erase(map.begin()+i-1, map.begin()+i+1);
    }

    return map;
}

bool BallMonitor::CompareAngles(const Angle& a1, const Angle& a2)
{
    return a1.val > a2.val;
}

BallMonitor::Angle BallMonitor::CreateAngle(double val, int id)
{
    Angle a = {val, id};
    return a;
}

double BallMonitor::AngleVertMirror(double angle)
{
    return angle >= 0 ? M_PI-angle : -M_PI-angle;
}

std::vector<double> BallMonitor::MergeVisibilityMaps(std::vector<double>& map1, std::vector<double>& map2)
{
    int i1 = 0, i2 = 0;
    int n1 = map1.size(), n2 = map2.size();
    std::vector<double> map;

    while (i1 < n1 && i2 < n2)
    {
        double a1 = map1[i1];
        double a2 = map2[i2];

        if (a1 <= a2)
        {
            if (i2 % 2 == 0)
                map.push_back(a1);
            i1++;
        }
        else
        {
            if (i1 % 2 == 0)
                map.push_back(a2);
            i2++;
        }
    }

    if (i1 < n1)
        map.insert(map.end(), map1.begin()+i1, map1.end());
    else if (i2 < n2)
        map.insert(map.end(), map2.begin()+i2, map2.end());

    //Remove doubles
    for (int i = map.size()-1 ; i >= 0 ; i -= 2)
    {
        if (map[i] == map[i-1])
            map.erase(map.begin()+i-1, map.begin()+i+1);
    }

    return map;
}


bool BallMonitor::ComputeLinearRegression(double *a, double *b, int precision) const
{
    PosTime tab[NB_POSTIME];
    int n, s;

    pthread_mutex_lock((pthread_mutex_t*)&m_ballMonitoringMtx);
    memcpy(tab, m_ballPosTime, sizeof(PosTime)*NB_POSTIME);
    n = std::min(std::max(precision, 2), m_nbBallPosTime);
    s = m_ballPosTimeInd;
    pthread_mutex_unlock((pthread_mutex_t*)&m_ballMonitoringMtx);

    double sumXi=0, sumXi2=0, sumYi=0, sumXiYi=0;
    for (int i=0 ; i < n ; i++)
    {
        int j = (s-i + NB_POSTIME) % NB_POSTIME;
        Position pos = m_coordCalibrer->NormalizePosition(tab[j].pos);
        double x = pos.GetX();
        double y = pos.GetY();
        sumXi += x;
        sumXi2 += x * x;
        sumYi += y;
        sumXiYi += y * x;
    }

    double k = n * sumXi2 - sumXi * sumXi;
    if (fabs(k) <= 1e-4)
        return false;

    if (b)
        *b = (sumXi2 * sumYi - sumXi * sumXiYi) / k;
    if (a)
        *a = (n * sumXiYi - sumXi * sumYi) / k;

    return true;
}

void BallMonitor::ResetPosTimeList()
{
    pthread_mutex_lock(&m_ballMonitoringMtx);
    m_nbBallPosTime = 1;
    pthread_mutex_unlock(&m_ballMonitoringMtx);
}

void* BallMonitor::BallMonitoringFn(void *data)
{
    BallMonitor *monitor = (BallMonitor*)data;
    Position pos;

    monitor->m_ballMonitoring = true;
    monitor->m_stopBallMonitoring = false;

    pthread_mutex_lock(&monitor->m_ballMonitoringMtx);
    monitor->m_ballPosTime[0].pos = monitor->m_mainBall->GetPos();
    monitor->m_ballPosTime[0].time = clock();
    monitor->m_nbBallPosTime = 1;
    monitor->m_ballPosTimeInd = 0;
    pthread_mutex_unlock(&monitor->m_ballMonitoringMtx);

    while (!monitor->m_stopBallMonitoring)
    {
        while (!monitor->m_stopBallMonitoring && (pos = monitor->m_mainBall->GetPos()).DistanceTo(monitor->m_ballPosTime[monitor->m_ballPosTimeInd].pos) < 0.025)
            usleep(1000);

        if (monitor->m_stopBallMonitoring)
            break;

        pthread_mutex_lock(&monitor->m_ballMonitoringMtx);
        monitor->m_ballPosTimeInd = (monitor->m_ballPosTimeInd + 1) % NB_POSTIME;
        monitor->m_nbBallPosTime = monitor->m_nbBallPosTime < NB_POSTIME ? monitor->m_nbBallPosTime+1 : monitor->m_nbBallPosTime;
        monitor->m_ballPosTime[monitor->m_ballPosTimeInd].pos = pos;
        monitor->m_ballPosTime[monitor->m_ballPosTimeInd].time = clock();
        pthread_mutex_unlock(&monitor->m_ballMonitoringMtx);
    }

    monitor->m_ballMonitoring = false;
    return NULL;
}

