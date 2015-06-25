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
    RefereeDisplay(eTeam team, BallMonitor *ballMonitor, CoordinatesCalibrer *coordCalibrer,
                   int screenW = 800, int screenH = 600, NewRoboControl **robots=NULL, RawBall *ball=NULL, const Interpreter::Map *map=NULL);
    ~RefereeDisplay();

    bool StartDisplay(NewRoboControl **robots=NULL, RawBall *ball=NULL, const Interpreter::Map *map=NULL);
    bool StopDisplay();
    bool IsActive() const;

    void DisplayPolygons(const PathFinder::PolygonsList& polygons);
    void DisplayPath(const PathFinder::Path path);

private:
    static void* RefDisplayFn(void *data);

    bool m_keepGoing;
    bool m_isDisplaying;
    pthread_t m_displayThread;
    int m_screenW, m_screenH;
    NewRoboControl *m_robots[6];
    RawBall *m_ball;
    eTeam m_team;
    BallMonitor *m_ballMonitor;
    CoordinatesCalibrer *m_coordCalibrer;
    MapDisplay *m_mapDisplay;
    const PathFinder::PolygonsList* m_polygons;
    PathFinder::Path m_path;

    void CreateMapDisplay(const Interpreter::Map *map);
    SDL_Rect PosToRect(Position pos, int w = 0, int h = 0);
    Position RectToPos(SDL_Rect rect);

};

#endif  //REFEREEDISPLAY_H
