#include "refereedisplay.h"

#define SCREENW 1080
#define SCREENH 810

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
    IMG_Init(IMG_INIT_PNG);
    SDL_Surface *screen = SDL_SetVideoMode(SCREENW, SCREENH, 32, SDL_SWSURFACE);
    SDL_Flip(screen);

    SDL_Surface *ballSurf = IMG_Load("../ball.png");
    if (!ballSurf)
        cout << "Unable to load ball bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(ballSurf, SDL_SRCCOLORKEY, SDL_MapRGB(ballSurf->format, 255, 255, 255));

    SDL_Surface *redRobotSurf = IMG_Load("../red_robot.png");
    if (!redRobotSurf)
        cout << "Unable to load red robot bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(redRobotSurf, SDL_SRCCOLORKEY, SDL_MapRGB(redRobotSurf->format, 255, 255, 255));

    SDL_Surface *blueRobotSurf = IMG_Load("../blue_robot.png");
    if (!blueRobotSurf)
        cout << "Unable to load blue robot bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(blueRobotSurf, SDL_SRCCOLORKEY, SDL_MapRGB(blueRobotSurf->format, 255, 255, 255));

    keepGoing = true;
    while (keepGoing)
    {
        SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 200, 0));

        SDL_Rect rect;
        if (ballSurf)
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()), ballSurf->w, ballSurf->h);
            SDL_BlitSurface(ballSurf, NULL, screen, &rect);
        }

        Position pos(0,0);
        if (PredictBallPosition(&pos))
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()));
            SDL_Rect rect2 = PosToRect(NormalizePosition(pos));
            DrawLine(screen, rect.x, rect.y, rect2.x, rect2.y, CreateColor(255,0,0));
        }

        if (blueRobotSurf)
        {
            for (int i=0 ; i < 3 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), blueRobotSurf->w, blueRobotSurf->h);
                SDL_BlitSurface(blueRobotSurf, NULL, screen, &rect);
            }
        }

        if (redRobotSurf)
        {
            for (int i=3 ; i < 6 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), redRobotSurf->w, redRobotSurf->h);
                SDL_BlitSurface(redRobotSurf, NULL, screen, &rect);
            }
        }

        SDL_Flip(screen);
        usleep(20000);
    }

    if (ballSurf)
        SDL_FreeSurface(ballSurf);
    if (redRobotSurf)
        SDL_FreeSurface(redRobotSurf);
    if (blueRobotSurf)
        SDL_FreeSurface(blueRobotSurf);

    IMG_Quit();
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

