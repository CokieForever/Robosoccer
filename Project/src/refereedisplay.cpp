#include "refereedisplay.h"
#include "interpreter.h"
#include "sdl_gfx/SDL_gfxPrimitives.h"
#include "log.h"
#include "teamrobot.h"
#include "geometry.h"

/**
 * @brief Standard constructor.
 *
 * Once the instance is created, you must call the @ref RefereeDisplay::StartDisplay() function to create the display's window.
 * Once the window is created, you must call the @ref RefereeDisplay::StopDisplay() function to destroy it before destroying the object.
 *
 * @param ballMonitor A ball monitoring system. The display uses it to display the ball.
 * @param coordCalibrer A coordinates calibrator.
 * @param screenW The width of the window you want, in pixels.
 * @param screenH The height of the window you want, in pixels.
 * @param robots An array containing the six robots. Can be NULL but in this case you will have to provide them when calling the @ref RefereeDisplay::StartDisplay() function.
 * @param interpreter The interpreter ruling the game. Can be NULL but in this case you will have to provide it when calling the @ref RefereeDisplay::StartDisplay() function.
 */
RefereeDisplay::RefereeDisplay(BallMonitor *ballMonitor, const CoordinatesCalibrer *coordCalibrer,
                               int screenW, int screenH, NewRoboControl **robots, const Interpreter *interpreter)
{
    m_keepGoing = true;
    m_isDisplaying = false;
    m_displayThread = NULL;
    m_screenW = screenW;
    m_screenH = screenH;

    if (robots)
        memcpy(m_robots, robots, sizeof(NewRoboControl*) * 6);
    else
        memset(m_robots, 0, sizeof(NewRoboControl*) * 6);

    m_interpreter = interpreter;
    m_ballMonitor = ballMonitor;
    m_coordCalibrer = coordCalibrer;
    m_mapDisplay = NULL;
    m_pathFinder = NULL;

    if (interpreter)
        m_team = interpreter->getMode().team;

    pthread_mutex_init(&m_pathMutex, NULL);
}

/**
 * @brief Standard destructor.
 *
 * This desctructor does not close the window and does not free the memory associated with the window.
 * Consequently, the @ref RefereeDisplay::StopDisplay() function must be called before the destructor, if a window was created.
 */
RefereeDisplay::~RefereeDisplay()
{
    if (m_mapDisplay)
        delete m_mapDisplay;
    pthread_mutex_destroy(&m_pathMutex);
}

/**
 * @brief Creates a window and launches a thread to keep it updated with the game data.
 *
 * After creating a window, the @ref RefereeDisplay::StopDisplay() function must be called before destroying the instance.
 *
 * @param robots The 6 robots to display. Can be NULL if there were provided in the @ref RefereeDisplay::RefereeDisplay() "constructor".
 * @param interpreter The interpreter ruling the game. Can be NULL if it was provided in the @ref RefereeDisplay::RefereeDisplay() "constructor".
 * @param map A map representing the obstacles in the field. Can be NULL. If provided, the map must not be deleted before the instance.
 * @return bool True if the window was actually created, False if not (missing data / already launched).
 */
bool RefereeDisplay::StartDisplay(NewRoboControl **robots, const Interpreter *interpreter, const Interpreter::Map *map)
{
    if (m_isDisplaying || !m_ballMonitor || !m_coordCalibrer)
        return false;

    if (robots)
        memcpy(m_robots, robots, sizeof(NewRoboControl*) * 6);
    if (interpreter)
    {
        m_interpreter = interpreter;
        m_team = interpreter->getMode().team;
    }

    CreateMapDisplay(map);

    pthread_create(&m_displayThread, NULL, RefDisplayFn, this);
    usleep(0.1e6);
    return true;
}

/**
 * @brief Destroys the display's window created by @ref RefereeDisplay::StartDisplay().
 *
 * @return bool True if the window was destroyed, False if not (not created yet).
 */
bool RefereeDisplay::StopDisplay()
{
    if (!m_isDisplaying)
        return false;
    m_keepGoing = false;
    pthread_join(m_displayThread, NULL);
    return true;
}

/**
 * @brief Tells if the window is created and active.
 *
 * @return bool True if the window is active, False if not.
 */
bool RefereeDisplay::IsActive() const
{
    return m_isDisplaying;
}

/**
 * @brief Adds the world of the provided @ref PathFinder "path finder" to the display.
 *
 * Only one path finder can be displayed at a time.
 * Note that displaying both an @ref Interpreter::Map "obstacles map" and a path finder world can make the display completely unreadable.
 *
 * @param pathFinder The path finder to display. It is not copied, so updates you make will be reported on the display.
 */
void RefereeDisplay::DisplayPathFinder(PathFinder *pathFinder)
{
    m_pathFinder = pathFinder;
}

/**
 * @brief Adds a @ref PathFinder::Path "path" to the display.
 *
 * Only one path can be displayed at a time.
 *
 * @param path The path to display.
 */
void RefereeDisplay::DisplayPath(const PathFinder::Path path)
{
    pthread_mutex_lock((pthread_mutex_t*)&m_pathMutex);
    m_path.clear();

    if (path)
    {
        int n = path->size();
        for (int i=0 ; i < n ; i++)
        {
            PathFinder::Point *pt = &((*path)[i]);
            m_path.push_back(PathFinder::CreatePoint(pt->x, pt->y));
        }
    }
    pthread_mutex_unlock((pthread_mutex_t*)&m_pathMutex);
}


/**
 * @brief Creates an internal @ref MapDisplay "display" for the given @ref Interpreter::Map "obstacles map".
 *
 * Only one map display can be used at a time. The old map display is deleted, if any.
 *
 * @param map The obstacles map.
 */
void RefereeDisplay::CreateMapDisplay(const Interpreter::Map *map)
{
    if (m_mapDisplay)
    {
        delete m_mapDisplay;
        m_mapDisplay = NULL;
    }

    m_mapDisplay = new MapDisplay(*map, m_screenW, m_screenH);
}

/**
 * @brief Displays the vertices links which are connected to the given @ref PathFinder::ConvexPolygon "polygon" in the world of the @ref PathFinder "path finder" of the display, if any.
 *
 * @param polygon The polygon.
 * @param screen The surface of the display's window.
 */
void RefereeDisplay::DisplayWeb(const PathFinder::ConvexPolygon& polygon, SDL_Surface *screen)
{
    for (PathFinder::PointsList::iterator it2 = ((PathFinder::ConvexPolygon&)polygon).points.begin() ; it2 != ((PathFinder::ConvexPolygon&)polygon).points.end() ; it2++)
    {
        PathFinder::Point *pt1 = *it2;
        double x1 = m_screenW * (pt1->x + 1) / 2;
        double y1 = m_screenH * (pt1->y + 1) / 2;
        for (PathFinder::PointsList::iterator it3 = pt1->visMap.begin() ; it3 != pt1->visMap.end() ; it3++)
        {
            PathFinder::Point *pt2 = *it3;
            double x2 = m_screenW * (pt2->x + 1) / 2;
            double y2 = m_screenH * (pt2->y + 1) / 2;
            DrawLine(screen, x1, y1, x2, y2, CreateColor(255, 255, 0));
        }

        if (m_pathFinder)
        {
            Position pos = m_coordCalibrer->NormalizePosition(m_robots[1]->GetPos());
            PathFinder::Point pt3 = PathFinder::CreatePoint(pos.GetX(), pos.GetY());
            if (m_pathFinder->CheckPointsVisibility(pt1, &pt3))
            {
                double x3 = m_screenW * (pt3.x + 1) / 2;
                double y3 = m_screenH * (pt3.y + 1) / 2;
                DrawLine(screen, x1, y1, x3, y3, CreateColor(255, 255, 0));
            }

            pos = m_coordCalibrer->NormalizePosition(GetBallPos());
            pt3 = PathFinder::CreatePoint(pos.GetX(), pos.GetY());
            if (m_pathFinder->CheckPointsVisibility(pt1, &pt3))
            {
                double x3 = m_screenW * (pt3.x + 1) / 2;
                double y3 = m_screenH * (pt3.y + 1) / 2;
                DrawLine(screen, x1, y1, x3, y3, CreateColor(255, 255, 0));
            }
        }
    }
}

/**
 * @brief Main display loop, designed to be launched in a separate thread.
 *
 * @param data A pointer to the @ref RefereeDisplay instance.
 */
void* RefereeDisplay::RefDisplayFn(void *data)
{
    RefereeDisplay *display = (RefereeDisplay*)data;
    display->m_isDisplaying = true;

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_Surface *screen = SDL_SetVideoMode(display->m_screenW, display->m_screenH, 32, SDL_SWSURFACE);
    SDL_Flip(screen);

    SDL_Surface *ballSurf = SDL_LoadBMP("../data/ball.bmp"), *ballSurfTr = NULL;
    if (!ballSurf)
        Log(string("Unable to load ball bmp: ") + SDL_GetError(), WARNING);
    else
    {
        double zoom = display->m_screenW/50 / (double)ballSurf->w;
        SDL_Surface *s = zoomSurface(ballSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(ballSurf);
            ballSurf = s;
        }

        ballSurfTr = SDL_CreateRGBSurfaceFrom(ballSurf->pixels, ballSurf->w, ballSurf->h, 32, ballSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Surface *redRobotSurf = SDL_LoadBMP("../data/red_robot.bmp"), *redRobotSurfTr = NULL;
    if (!redRobotSurf)
        Log(string("Unable to load red robot bmp: ") + SDL_GetError(), WARNING);
    else
    {
        double zoom = display->m_screenW/25 / (double)redRobotSurf->w;
        SDL_Surface *s = zoomSurface(redRobotSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(redRobotSurf);
            redRobotSurf = s;
        }

        redRobotSurfTr = SDL_CreateRGBSurfaceFrom(redRobotSurf->pixels, redRobotSurf->w, redRobotSurf->h, 32, redRobotSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Surface *blueRobotSurf = SDL_LoadBMP("../data/blue_robot.bmp"), *blueRobotSurfTr = NULL;
    if (!blueRobotSurf)
        Log(string("Unable to load blue robot bmp: ") + SDL_GetError(), WARNING);
    else
    {
        double zoom = display->m_screenW/25 / (double)blueRobotSurf->w;
        SDL_Surface *s = zoomSurface(blueRobotSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(blueRobotSurf);
            blueRobotSurf = s;
        }

        blueRobotSurfTr = SDL_CreateRGBSurfaceFrom(blueRobotSurf->pixels, blueRobotSurf->w, blueRobotSurf->h, 32, blueRobotSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Rect rect = {0,0,0,0};
    DragDrop robotsDD[6] = {CreateDragDrop(rect, display->m_team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, display->m_team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, display->m_team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, display->m_team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr),
                            CreateDragDrop(rect, display->m_team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr),
                            CreateDragDrop(rect, display->m_team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr)};

    Position gotoOrders[6] = {Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10)};

    SDL_Event event;
    display->m_keepGoing = true;
    while (display->m_keepGoing)
    {
        event.type = SDL_NOEVENT;
        SDL_PollEvent(&event);

        if (event.type == SDL_QUIT)
            break;

        int n;
        eSide ourSide = display->m_interpreter->getMode().our_side;
        SDL_Rect r;

        #ifdef PATHPLANNING_ASTAR
        SDL_Surface *bgSurf = display->m_mapDisplay ? display->m_mapDisplay->UpdateDisplay() : NULL;
        if (bgSurf)
            SDL_BlitSurface(bgSurf, NULL, screen, NULL);
        else
            SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0,255,0));
        #else
        SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0,255,0));
        #endif

        #if defined(PATHPLANNING_POLYGONS) && !defined(PATHPLANNING_ASTAR)

        //Display path finder polygons
        if (display->m_pathFinder)
        {
            PathFinder::PolygonsList polygons = display->m_pathFinder->GetPolygonsCopy();
            for (PathFinder::PolygonsList::iterator it = polygons.begin() ; it != polygons.end() ; it++)
            {
                PathFinder::ConvexPolygon *polygon = *it;
                int n = polygon->points.size();
                Sint16 *vx = new Sint16[n];
                Sint16 *vy = new Sint16[n];

                for (int i=0 ; i < n ; i++)
                {
                    SDL_Rect r = display->PosToRect(Position(polygon->points[i]->x, polygon->points[i]->y));
                    vx[i] = std::max(0, std::min(display->m_screenW-1, (int)r.x));
                    vy[i] = std::max(0, std::min(display->m_screenH-1, (int)r.y));
                }

                filledPolygonRGBA(screen, vx, vy, n, 255, 0, 0, 255);
                delete vx;
                delete vy;

                //display->DisplayWeb(*polygon, screen);
            }
        }

        //Display current path
        pthread_mutex_lock(&(display->m_pathMutex));
        n = display->m_path.size();
        if (n > 1)
        {
            PathFinder::Point *point = &(display->m_path[0]);
            SDL_Rect r1 = display->PosToRect(Position(point->x, point->y));
            for (int i=1 ; i < n ; i++)
            {
                PathFinder::Point *next = &(display->m_path[i]);
                SDL_Rect r2 = display->PosToRect(Position(next->x, next->y));

                DrawLine(screen, r1.x, r1.y, r2.x, r2.y, CreateColor(255,0,0));

                r1 = r2;
            }
        }
        pthread_mutex_unlock(&(display->m_pathMutex));

        #endif

        Position ballPos = display->GetBallPos();
        Position ballPos_n = display->m_coordCalibrer->NormalizePosition(ballPos);
        if (ballSurfTr)
        {
            rect = display->PosToRect(ballPos_n, ballSurfTr->w, ballSurfTr->h);
            SDL_BlitSurface(ballSurfTr, NULL, screen, &rect);
        }

        //Display visibility map for the ball
        std::vector<double> map = display->m_ballMonitor->ComputeVisibilityMap(2, (const NewRoboControl**)display->m_robots, ourSide, display->m_coordCalibrer);
        n = map.size();
        SDL_Rect ballPosR = display->PosToRect(ballPos_n);
        for (int i=0 ; i < n ; i += 2)
        {
            double x, y;
            ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), map[i], 4, &x, &y);
            SDL_Rect r2 = display->PosToRect(Position(x, y));

            ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), map[i+1], 4, &x, &y);
            SDL_Rect r3 = display->PosToRect(Position(x, y));

            filledTrigonRGBA(screen, ballPosR.x, ballPosR.y, r2.x, r2.y, r3.x, r3.y, 255, 255, 0, 100);
        }

        //Display optimal ball direction
        double x, y;
        double dir = BallMonitor::GetBestDirection(map, ourSide);
        ComputeVectorEnd(ballPos_n.GetX(), ballPos_n.GetY(), dir, 4, &x, &y);
        r = display->PosToRect(Position(x, y));
        DrawLine(screen, ballPosR.x, ballPosR.y, r.x, r.y, CreateColor(255,255,255));

        //Display ball trajectory prevision
        double a, b;
        if (display->m_ballMonitor->PredictBallPosition(&a, &b, 4))
        {
            double isectX[2], isectY[2];
            if (GetLineRectIntersections(-0.95, -0.95, 0.95, 0.95, a, b, isectX, isectY))
            {
                r = display->PosToRect(Position(isectX[0], isectY[0]));
                SDL_Rect r2 = display->PosToRect(Position(isectX[1], isectY[1]));
                DrawLine(screen, r.x, r.y, r2.x, r2.y, CreateColor(255,0,0));
            }
        }

        Position goalPos_n(ourSide == LEFT_SIDE ? +1 : -1, 0);
        Position goalPos = display->m_coordCalibrer->UnnormalizePosition(goalPos_n);

        for (int i=0 ; i < 6 ; i++)
        {
            SDL_Surface *robotSurf = ((display->m_team == BLUE_TEAM) ^ (i < 3)) ? redRobotSurfTr : blueRobotSurfTr;
            if (robotSurf)
            {
                Position robotPos = display->m_robots[i]->GetPos();
                Position robotPos_n = display->m_coordCalibrer->NormalizePosition(robotPos);
                robotsDD[i].area = display->PosToRect(robotPos_n, robotSurf->w, robotSurf->h);
                rect = robotsDD[i].area;
                SDL_BlitSurface(robotSurf, NULL, screen, &rect);

                SDL_Color color = CreateColor(0,0,0);
                SDL_Rect robotR = display->PosToRect(robotPos_n);
                if (i < 3 && ((TeamRobot*)(display->m_robots[i]))->ShouldKick(ballPos, goalPos))
                {
                    color = CreateColor(255, 255, 255);
                    SDL_Rect goalR = display->PosToRect(goalPos_n);
                    DrawLine(screen, robotR.x, robotR.y, goalR.x, goalR.y, color);
                }

                double phi = display->m_robots[i]->GetPhi().Get();
                double endX, endY;
                ComputeVectorEnd(robotPos.GetX(), robotPos.GetY(), phi, 0.15, &endX, &endY);
                SDL_Rect r = display->PosToRect(display->m_coordCalibrer->NormalizePosition(Position(endX, endY)));
                DrawLine(screen, robotR.x, robotR.y, r.x, r.y, color);

                DragDropStatus status = ManageDragDrop(&(robotsDD[i]), event);

                if (status == DDS_DROPPED)
                {
                    gotoOrders[i] = display->m_coordCalibrer->UnnormalizePosition(display->RectToPos(GetMousePos()));
                    display->m_robots[i]->GotoXY(gotoOrders[i].GetX(), gotoOrders[i].GetY(), 130);
                }
                else if (status == DDS_DRAGGED)
                {
                    SDL_Rect mousePos = GetMousePos();
                    DrawLine(screen, robotsDD[i].area.x + robotSurf->w/2, robotsDD[i].area.y + robotSurf->h/2, mousePos.x, mousePos.y, CreateColor(255,255,0));
                }

                if (gotoOrders[i].GetX() > -10)
                {
                    if (display->m_robots[i]->GetPos().DistanceTo(gotoOrders[i]) <= 0.1)
                        gotoOrders[i].SetX(-10);
                    else
                    {
                        rect = display->PosToRect(display->m_coordCalibrer->NormalizePosition(gotoOrders[i]));
                        DrawLine(screen, robotsDD[i].area.x + robotSurf->w/2, robotsDD[i].area.y + robotSurf->h/2, rect.x, rect.y, CreateColor(255,128,0));
                    }
                }
            }
        }

        SDL_Flip(screen);
        usleep(20000);
    }

    if (ballSurf)
        SDL_FreeSurface(ballSurf);
    if (ballSurfTr)
        SDL_FreeSurface(ballSurfTr);
    if (redRobotSurf)
        SDL_FreeSurface(redRobotSurf);
    if (redRobotSurfTr)
        SDL_FreeSurface(redRobotSurfTr);
    if (blueRobotSurf)
        SDL_FreeSurface(blueRobotSurf);
    if (blueRobotSurfTr)
        SDL_FreeSurface(blueRobotSurfTr);

    SDL_Quit();

    display->m_isDisplaying = false;
    return NULL;
}

/**
 * @brief Converts a NORMALIZED position (see @ref CoordinatesCalibrer for more info) to screen coordinates.
 *
 * @param pos The position to convert.
 * @param w w/2 = horizontal offset (useful when displaying pictures of width w)
 * @param h h/2 = vertical offset (useful when displaying pictures of heigth h)
 * @return SDL_Rect The screen coordinates.
 */
SDL_Rect RefereeDisplay::PosToRect(Position pos, int w, int h)
{
    SDL_Rect rect = {(pos.GetX()+1)/2 * (m_screenW-1) - w/2, (pos.GetY()+1)/2 * (m_screenH-1) - h/2, w, h};
    return rect;
}

/**
 * @brief Converts screen coordinates to a normalized position (see @ref CoordinatesCalibrer for more info).
 *
 * @param rect The screen coordinates.
 * @return Position The normalized position.
 */
Position RefereeDisplay::RectToPos(SDL_Rect rect)
{
    return Position(2 * rect.x / (double)(m_screenW-1) - 1, 2 * rect.y / (double)(m_screenH-1) - 1);
}

/**
 * @brief Gets the unnormalized position of the ball, extracted from the internal
 * @ref BallMonitor "ball monitoring system" (provided in the @ref RefereeDisplay::RefereeDisplay "constructor").
 *
 * @return Position The ball's position.
 */
Position RefereeDisplay::GetBallPos()
{
    Position ballPos;
    m_ballMonitor->GetBallPosition(&ballPos);
    return ballPos;
}
