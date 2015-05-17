#include <SDL.h>
#include "referee.h"
#include "robo_control.h"
#include "coordinates.h"

typedef struct
{
    RoboControl *robots[6];
    RawBall *ball;
} AllData;

bool StartRefereeDisplay(RoboControl **robots, RawBall *ball);
bool StopRefereeDisplay();
