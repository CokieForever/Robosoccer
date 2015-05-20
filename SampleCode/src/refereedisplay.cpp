#include "refereedisplay.h"

#define SCREENW 800
#define SCREENH 600

static bool keepGoing = true;
static bool isDisplaying = false;
static pthread_t displayThread;

static void* RefDisplayFn(void *data);
static SDL_Rect PosToRect(Position pos, int w = 0, int h = 0);
static Position RectToPos(SDL_Rect rect);

bool StartRefereeDisplay(RoboControl **robots, RawBall *ball, eTeam team)
{
    if (isDisplaying)
        return false;

    AllData *data = (AllData*)malloc(sizeof(AllData));
    for (int i=0 ; i < 6 ; i++)
        data->robots[i] = robots[i];
    data->ball = ball;
    data->team = team;

    pthread_create(&displayThread, NULL, RefDisplayFn, data);
    usleep(0.1e6);

    return true;
}

bool StopRefereeDisplay()
{
    if (!isDisplaying)
        return false;
    keepGoing = false;
    pthread_join(displayThread, NULL);
    return true;
}



static void* RefDisplayFn(void *data)
{
    AllData *allData = (AllData*)data;
    isDisplaying = true;

    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Surface *screen = SDL_SetVideoMode(SCREENW, SCREENH, 32, SDL_SWSURFACE);
    SDL_Flip(screen);

    SDL_Surface *ballSurf = SDL_LoadBMP("../ball.bmp"), *ballSurfTr = NULL;
    if (!ballSurf)
        cout << "Unable to load ball bmp: " << SDL_GetError() << endl;
    else
    {
        double zoom = SCREENW/50 / (double)ballSurf->w;
        SDL_Surface *s = zoomSurface(ballSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(ballSurf);
            ballSurf = s;
        }

        ballSurfTr = SDL_CreateRGBSurfaceFrom(ballSurf->pixels, ballSurf->w, ballSurf->h, 32, ballSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Surface *redRobotSurf = SDL_LoadBMP("../red_robot.bmp"), *redRobotSurfTr = NULL;
    if (!redRobotSurf)
        cout << "Unable to load red robot bmp: " << SDL_GetError() << endl;
    else
    {
        double zoom = SCREENW/25 / (double)redRobotSurf->w;
        SDL_Surface *s = zoomSurface(redRobotSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(redRobotSurf);
            redRobotSurf = s;
        }

        redRobotSurfTr = SDL_CreateRGBSurfaceFrom(redRobotSurf->pixels, redRobotSurf->w, redRobotSurf->h, 32, redRobotSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Surface *blueRobotSurf = SDL_LoadBMP("../blue_robot.bmp"), *blueRobotSurfTr = NULL;
    if (!blueRobotSurf)
        cout << "Unable to load blue robot bmp: " << SDL_GetError() << endl;
    else
    {
        double zoom = SCREENW/25 / (double)blueRobotSurf->w;
        SDL_Surface *s = zoomSurface(blueRobotSurf, zoom, zoom, 1);
        if (s)
        {
            SDL_FreeSurface(blueRobotSurf);
            blueRobotSurf = s;
        }

        blueRobotSurfTr = SDL_CreateRGBSurfaceFrom(blueRobotSurf->pixels, blueRobotSurf->w, blueRobotSurf->h, 32, blueRobotSurf->pitch, 0xff000000, 0x00ff0000, 0x0000ff00, 0x000000ff);
    }

    SDL_Rect rect = {0,0,0,0};
    DragDrop robotsDD[6] = {CreateDragDrop(rect, allData->team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, allData->team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, allData->team == BLUE_TEAM ? blueRobotSurfTr : redRobotSurfTr),
                            CreateDragDrop(rect, allData->team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr),
                            CreateDragDrop(rect, allData->team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr),
                            CreateDragDrop(rect, allData->team == BLUE_TEAM ? redRobotSurfTr : blueRobotSurfTr)};

    Position gotoOrders[6] = {Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10), Position(-10,-10)};

    SDL_Event event;
    keepGoing = true;
    while (keepGoing)
    {
        event.type = SDL_NOEVENT;
        SDL_PollEvent(&event);

        SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 200, 0));

        if (ballSurfTr)
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()), ballSurfTr->w, ballSurfTr->h);
            SDL_BlitSurface(ballSurfTr, NULL, screen, &rect);
        }

        Position pos(0,0);
        if (PredictBallPosition(&pos))
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()));
            SDL_Rect rect2 = PosToRect(NormalizePosition(pos));
            DrawLine(screen, rect.x, rect.y, rect2.x, rect2.y, CreateColor(255,0,0));
        }

        for (int i=0 ; i < 6 ; i++)
        {
            SDL_Surface *robotSurf = ((allData->team == BLUE_TEAM) ^ (i < 3)) ? redRobotSurfTr : blueRobotSurfTr;
            if (robotSurf)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), robotSurf->w, robotSurf->h);
                SDL_BlitSurface(robotSurf, NULL, screen, &rect);

                robotsDD[i].area = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), robotSurf->w, robotSurf->h);
                DragDropStatus status = ManageDragDrop(&(robotsDD[i]), event);

                if (status == DDS_DROPPED)
                {
                    gotoOrders[i] = UnnormalizePosition(RectToPos(GetMousePos()));
                    allData->robots[i]->GotoPos(gotoOrders[i]);
                }
                else if (status == DDS_DRAGGED)
                {
                    rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()));
                    SDL_Rect mousePos = GetMousePos();
                    DrawLine(screen, rect.x, rect.y, mousePos.x, mousePos.y, CreateColor(255,255,0));
                }

                if (gotoOrders[i].GetX() > -10)
                {
                    if (allData->robots[i]->GetPos().DistanceTo(gotoOrders[i]) <= 0.1)
                        gotoOrders[i].SetX(-10);
                    else
                    {
                        rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()));
                        SDL_Rect rect2 = PosToRect(NormalizePosition(gotoOrders[i]));
                        DrawLine(screen, rect.x, rect.y, rect2.x, rect2.y, CreateColor(255,128,0));
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

    isDisplaying = false;
    free(allData);
    return NULL;
}

static SDL_Rect PosToRect(Position pos, int w, int h)
{
    SDL_Rect rect = {(pos.GetX()+1)/2 * SCREENW - w/2, (pos.GetY()+1)/2 * SCREENH - h/2, w, h};
    return rect;
}

static Position RectToPos(SDL_Rect rect)
{
    return Position(2 * rect.x / (double)SCREENW - 1, 2 * rect.y / (double)SCREENH - 1);
}
