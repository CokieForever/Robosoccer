#ifndef BALLMONITOR_H
#define BALLMONITOR_H

#include <pthread.h>
#include <time.h>
#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "coordinates.h"
#include <vector>
#include <queue>

class BallMonitor
{

public:

    static const int NB_POSTIME  = 10;

    typedef struct
    {
        double x, y;
    } Direction;

    typedef struct
    {
        Position pos;
        clock_t time;
    } PosTime;

    static std::vector<double> ComputeVisibilityMap(int maxLevel, Position pos, const Position *robotPos, int nbPos, eSide ourSide);
    static std::vector<double> ComputeVisibilityMap(Position pos, const Position *robotPos, int nbPos, eSide ourSide);
    static double GetBestDirection(std::vector<double> visibilityMap, eSide ourSide);

    BallMonitor(CoordinatesCalibrer *coordCalibrer, RawBall *ball = NULL);

    bool StartMonitoring(RawBall *ball = NULL);
    bool StopMonitoring();
    bool GetBallPosition(Position *pos) const;
    bool GetBallDirection(Direction *dir) const;
    bool PredictBallPosition(double *a, double *b, int precision);
    bool IsBallMoving() const;

    std::vector<double> ComputeVisibilityMap(int maxLevel, const NewRoboControl* robot[6], eSide ourSide, const CoordinatesCalibrer *coordCalib) const;
    std::vector<double> ComputeVisibilityMap(const NewRoboControl* robot[6], eSide ourSide, const CoordinatesCalibrer *coordCalib) const;

private:
    struct Angle
    {
        double val;
        int id;
    };

    static void* BallMonitoringFn(void *data);
    static void* BallFollowingFn(void *data);

    typedef bool (*CompareFn)(const Angle&, const Angle&);

    static bool CompareAngles(const Angle& a1, const Angle& a2);
    static Angle CreateAngle(double val, int id);
    static double AngleVertMirror(double angle);
    static std::vector<double> MergeVisibilityMaps(std::vector<double>& map1, std::vector<double>& map2);
    static std::vector<double> AnglesToMap(std::priority_queue<Angle, std::vector<Angle>, CompareFn> angles, double minAngle, double maxAngle);

    RawBall *m_mainBall;
    bool m_stopBallMonitoring;
    bool m_ballMonitoring;
    pthread_mutex_t m_ballMonitoringMtx;
    pthread_t m_ballMonitoringThread;
    bool m_ballFollowing;
    pthread_t m_ballFollowingThread;
    bool m_stopBallFollowing ;
    PosTime m_ballPosTime[NB_POSTIME];
    int m_ballPosTimeInd;
    int m_nbBallPosTime;
    NewRoboControl *m_followerRobot;
    CoordinatesCalibrer *m_coordCalibrer;

    void ResetPosTimeList();
    bool ComputeLinearRegression(double *a, double *b, int precision = 2) const;

};

#endif // BALLMONITOR_H
