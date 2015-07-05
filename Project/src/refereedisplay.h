#ifndef REFEREEDISPLAY_H
#define REFEREEDISPLAY_H

#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <sdl_gfx/SDL_rotozoom.h>
#include "referee.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "sdlutilities.h"
#include "mapdisplay.h"
#include "interpreter.h"
#include "pathfinder.h"
#include <vector>

class RefereeDisplay
{

public:
    RefereeDisplay(BallMonitor *ballMonitor, const CoordinatesCalibrer *coordCalibrer,
                   int screenW = 800, int screenH = 600, NewRoboControl **robots=NULL, const Interpreter *interpreter = NULL);
    ~RefereeDisplay();

    bool StartDisplay(NewRoboControl **robots=NULL, const Interpreter *interpreter = NULL, const Interpreter::Map *map=NULL);
    bool StopDisplay();
    bool IsActive() const;

    void DisplayPathFinder(PathFinder *pathFinder);
    void DisplayPath(const PathFinder::Path path);

private:
    static void* RefDisplayFn(void *data);

    bool m_keepGoing;
    bool m_isDisplaying;
    pthread_t m_displayThread;
    int m_screenW, m_screenH;
    NewRoboControl *m_robots[6];
    const Interpreter *m_interpreter;
    eTeam m_team;
    BallMonitor *m_ballMonitor;
    const CoordinatesCalibrer *m_coordCalibrer;
    MapDisplay *m_mapDisplay;
    std::vector<PathFinder::Point> m_path;
    PathFinder *m_pathFinder;
    pthread_mutex_t m_pathMutex;

    void CreateMapDisplay(const Interpreter::Map *map);
    void DisplayWeb(const PathFinder::ConvexPolygon& polygon, SDL_Surface *screen);
    SDL_Rect PosToRect(Position pos, int w = 0, int h = 0);
    Position RectToPos(SDL_Rect rect);
    Position GetBallPos();

};

#endif  //REFEREEDISPLAY_H
