#include <SDL.h>
#include <SDL_image.h>
#include <sdl_gfx/SDL_rotozoom.h>
#include "referee.h"
#include "robo_control.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "sdlutilities.h"

typedef struct
{
    RoboControl *robots[6];
    RawBall *ball;
    eTeam team;
} AllData;

bool StartRefereeDisplay(RoboControl **robots, RawBall *ball, eTeam team);
bool StopRefereeDisplay();
