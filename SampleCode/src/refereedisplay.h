#include <SDL.h>
#include <SDL_image.h>
#include "referee.h"
#include "robo_control.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "sdlutilities.h"

typedef struct
{
    RoboControl *robots[6];
    RawBall *ball;
} AllData;

bool StartRefereeDisplay(RoboControl **robots, RawBall *ball);
bool StopRefereeDisplay();
