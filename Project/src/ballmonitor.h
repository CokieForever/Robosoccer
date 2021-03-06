#ifndef BALLMONITOR_H
#define BALLMONITOR_H

#include <pthread.h>
#include <time.h>
#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "coordinates.h"
#include <vector>
#include <queue>

/**
 * @brief Class used as a wrapper for a "RawBall" instance.
 *
 * This class provides useful functions to do some computations relative to a given "RawBall".
 * For example you can use it to retrieve the ball position and speed, or to predict the ball trajectory.
 * The instance must be initialized via the constructor @ref BallMonitor::BallMonitor()
 * and then started with the @ref BallMonitor::StartMonitoring() function. When you do not need it anymore,
 * do not forget to stop it with the @ref BallMonitor::StopMonitoring() function.
 */
class BallMonitor
{

public:

    static const int NB_POSTIME  = 10; /**< Number of past ball positions to be kept in memory. */

    /**
     * @brief This structure contains the direction coordinates of the ball
     *
     */
    typedef struct
    {
        double x;   /**< x-coordinate */
        double y;   /**< y-coordinate */
    } Direction;

    /**
     * @brief This structure contains the position of the ball and the given time.
     *
     */
    typedef struct
    {
        Position pos;   /**< Ball position */
        clock_t time;   /**< Time at which the position was registered */
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
    /**
     * @brief Describes an angle.
     *
     */
    struct Angle
    {
        double val; /**< The angle value (in radians) */
        int id;     /**< The ID of the angle */
    };

    static void* BallMonitoringFn(void *data);
    static void* BallFollowingFn(void *data);

    /**
     * @brief
     *
     */
    typedef bool (*CompareFn)(const Angle&, const Angle&);

    static bool CompareAngles(const Angle& a1, const Angle& a2);
    static Angle CreateAngle(double val, int id);
    static double AngleVertMirror(double angle);
    static std::vector<double> MergeVisibilityMaps(std::vector<double>& map1, std::vector<double>& map2);
    static std::vector<double> AnglesToMap(std::priority_queue<Angle, std::vector<Angle>, CompareFn> angles, double minAngle, double maxAngle);

    RawBall *m_mainBall;                    /**< TODO */
    bool m_stopBallMonitoring;              /**< TODO */
    bool m_ballMonitoring;                  /**< TODO */
    pthread_mutex_t m_ballMonitoringMtx;    /**< TODO */
    pthread_t m_ballMonitoringThread;       /**< TODO */
    bool m_ballFollowing;                   /**< TODO */
    pthread_t m_ballFollowingThread;        /**< TODO */
    bool m_stopBallFollowing ;              /**< TODO */
    PosTime m_ballPosTime[NB_POSTIME];      /**< TODO */
    int m_ballPosTimeInd;                   /**< TODO */
    int m_nbBallPosTime;                    /**< TODO */
    NewRoboControl *m_followerRobot;        /**< TODO */
    CoordinatesCalibrer *m_coordCalibrer;   /**< TODO */

    void ResetPosTimeList();
    bool ComputeLinearRegression(double *a, double *b, int precision = 2) const;

};

#endif // BALLMONITOR_H
