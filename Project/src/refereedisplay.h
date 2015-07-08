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

/**
 * @brief
 *
 */
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

    bool m_keepGoing;                               /**< TODO */
    bool m_isDisplaying;                            /**< TODO */
    pthread_t m_displayThread;                      /**< TODO */
    int m_screenW, m_screenH;                       /**< TODO */
    NewRoboControl *m_robots[6];                    /**< TODO */
    const Interpreter *m_interpreter;               /**< TODO */
    eTeam m_team;                                   /**< TODO */
    BallMonitor *m_ballMonitor;                     /**< TODO */
    const CoordinatesCalibrer *m_coordCalibrer;     /**< TODO */
    MapDisplay *m_mapDisplay;                       /**< TODO */
    std::vector<PathFinder::Point> m_path;          /**< TODO */
    PathFinder *m_pathFinder;                       /**< TODO */
    pthread_mutex_t m_pathMutex;                    /**< TODO */

    void CreateMapDisplay(const Interpreter::Map *map);
    void DisplayWeb(const PathFinder::ConvexPolygon& polygon, SDL_Surface *screen);
    SDL_Rect PosToRect(Position pos, int w = 0, int h = 0);
    Position RectToPos(SDL_Rect rect);
    Position GetBallPos();

};

#endif  //REFEREEDISPLAY_H
