#include "refereedisplay.h"
#include "interpreter.h"


RefereeDisplay::RefereeDisplay(eTeam team, BallMonitor *ballMonitor, CoordinatesCalibrer *coordCalibrer, int screenW, int screenH,
                               NewRoboControl **robots, RawBall *ball, const Interpreter::Map *map)
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

    m_ball = ball;
    m_team = team;
    m_ballMonitor = ballMonitor;
    m_coordCalibrer = coordCalibrer;
    m_mapDisplay = NULL;

    CreateMapDisplay(map);
}

RefereeDisplay::~RefereeDisplay()
{
    if (m_mapDisplay)
        delete m_mapDisplay;
}

bool RefereeDisplay::StartDisplay(NewRoboControl **robots, RawBall *ball, const Interpreter::Map *map)
{
    if (m_isDisplaying || !m_ballMonitor || !m_coordCalibrer)
        return false;

    if (robots)
        memcpy(m_robots, robots, sizeof(NewRoboControl*) * 6);
    if (ball)
        m_ball = ball;
    if (map)
        CreateMapDisplay(map);

    pthread_create(&m_displayThread, NULL, RefDisplayFn, this);
    usleep(0.1e6);return StartDisplay(robots, ball);
}

bool RefereeDisplay::StopDisplay()
{
    if (!m_isDisplaying)
        return false;
    m_keepGoing = false;
    pthread_join(m_displayThread, NULL);
    return true;
}

bool RefereeDisplay::IsActive() const
{
    return m_isDisplaying;
}


void RefereeDisplay::CreateMapDisplay(const Interpreter::Map *map)
{
    if (m_mapDisplay)
    {
        delete m_mapDisplay;
        m_mapDisplay = NULL;
    }

    m_mapDisplay = new MapDisplay(*map, m_screenW, m_screenH);
}

void* RefereeDisplay::RefDisplayFn(void *data)
{
    RefereeDisplay *display = (RefereeDisplay*)data;
    display->m_isDisplaying = true;

    SDL_Init(SDL_INIT_EVERYTHING);

    SDL_Surface *screen = SDL_SetVideoMode(display->m_screenW, display->m_screenH, 32, SDL_SWSURFACE);
    SDL_Flip(screen);

    SDL_Surface *ballSurf = SDL_LoadBMP("../data/ball.bmp"), *ballSurfTr = NULL;
    if (!ballSurf)
        cout << "Unable to load ball bmp: " << SDL_GetError() << endl;
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
        cout << "Unable to load red robot bmp: " << SDL_GetError() << endl;
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
        cout << "Unable to load blue robot bmp: " << SDL_GetError() << endl;
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
    SDL_Surface *bgSurf = NULL;
    display->m_keepGoing = true;
    while (display->m_keepGoing)
    {
        event.type = SDL_NOEVENT;
        SDL_PollEvent(&event);

        if (event.type == SDL_QUIT)
            break;

        bgSurf = display->m_mapDisplay ? display->m_mapDisplay->UpdateDisplay() : NULL;
        if (bgSurf)
            SDL_BlitSurface(bgSurf, NULL, screen, NULL);
        else
            SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0,255,0));

        if (ballSurfTr)
        {
            rect = display->PosToRect(display->m_coordCalibrer->NormalizePosition(display->m_ball->GetPos()), ballSurfTr->w, ballSurfTr->h);
            SDL_BlitSurface(ballSurfTr, NULL, screen, &rect);
        }

        Position pos(0,0);
        if (display->m_ballMonitor->PredictBallPosition(&pos, 5))
        {
            rect = display->PosToRect(display->m_coordCalibrer->NormalizePosition(display->m_ball->GetPos()));
            SDL_Rect rect2 = display->PosToRect(display->m_coordCalibrer->NormalizePosition(pos));
            DrawLine(screen, rect.x, rect.y, rect2.x, rect2.y, CreateColor(255,0,0));
        }

        for (int i=0 ; i < 6 ; i++)
        {
            SDL_Surface *robotSurf = ((display->m_team == BLUE_TEAM) ^ (i < 3)) ? redRobotSurfTr : blueRobotSurfTr;
            if (robotSurf)
            {
                robotsDD[i].area = display->PosToRect(display->m_coordCalibrer->NormalizePosition(display->m_robots[i]->GetPos()), robotSurf->w, robotSurf->h);
                rect = robotsDD[i].area;
                SDL_BlitSurface(robotSurf, NULL, screen, &rect);

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

SDL_Rect RefereeDisplay::PosToRect(Position pos, int w, int h)
{
    SDL_Rect rect = {(pos.GetX()+1)/2 * m_screenW - w/2, (pos.GetY()+1)/2 * m_screenH - h/2, w, h};
    return rect;
}

Position RefereeDisplay::RectToPos(SDL_Rect rect)
{
    return Position(2 * rect.x / (double)m_screenW - 1, 2 * rect.y / (double)m_screenH - 1);
}
