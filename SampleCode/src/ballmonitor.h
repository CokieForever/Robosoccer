#ifndef BALLMONITOR_H
#define BALLMONITOR_H

#include <pthread.h>
#include <time.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "coordinates.h"
#include "robotmonitor.h"

typedef struct
{
    double x, y;
} Direction;

typedef struct
{
    Position pos;
    clock_t time;
} PosTime;

bool StartBallMonitoring(RawBall *ball);
bool StopBallMonitoring();
bool GetBallPosition(Position *pos);
bool GetBallDirection(Direction *dir);
bool PredictBallPosition(Position *pos, bool precise=false, int minPrecision = 2);
bool IsBallMoving();
bool StartBallFollowing(RoboControl *robo);
bool StopBallFollowing();
bool IsBallFollowingStarted();

#endif // BALLMONITOR_H
