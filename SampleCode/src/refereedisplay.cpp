#include "refereedisplay.h"

#define SCREENW 800
#define SCREENH 600

static bool keepGoing = true;
static bool isDisplaying = false;
static pthread_t displayThread;

static void* RefDisplayFn(void *data);
static SDL_Rect PosToRect(Position pos, int w = 0, int h = 0);

bool StartRefereeDisplay(RoboControl **robots, RawBall *ball)
{
    if (isDisplaying)
        return false;

    AllData *data = (AllData*)malloc(sizeof(AllData));
    for (int i=0 ; i < 6 ; i++)
        data->robots[i] = robots[i];
    data->ball = ball;

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

    keepGoing = true;
    while (keepGoing)
    {
        SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 200, 0));

        SDL_Rect rect;
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

        if (blueRobotSurfTr)
        {
            for (int i=0 ; i < 3 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), blueRobotSurfTr->w, blueRobotSurfTr->h);
                SDL_BlitSurface(blueRobotSurfTr, NULL, screen, &rect);
            }
        }

        if (redRobotSurfTr)
        {
            for (int i=3 ; i < 6 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), redRobotSurfTr->w, redRobotSurfTr->h);
                SDL_BlitSurface(redRobotSurfTr, NULL, screen, &rect);
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
    SDL_Rect rect = {(pos.GetX()+1)/2 * SCREENW - w/2, (pos.GetY()+1)/2 * SCREENH - h/2, 0, 0};
    return rect;
}

