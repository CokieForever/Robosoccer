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
    SDL_Surface *screen = SDL_SetVideoMode(SCREENW, SCREENH, 32, SDL_SWSURFACE);
    SDL_Flip(screen);

    SDL_Surface *ball = SDL_LoadBMP("../ball.bmp");
    if (!ball)
        cout << "Unable to load ball bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(ball, SDL_SRCCOLORKEY, SDL_MapRGB(ball->format, 255, 255, 255));

    SDL_Surface *redRobot = SDL_LoadBMP("../red_robot.bmp");
    if (!redRobot)
        cout << "Unable to load red robot bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(redRobot, SDL_SRCCOLORKEY, SDL_MapRGB(redRobot->format, 255, 255, 255));

    SDL_Surface *blueRobot = SDL_LoadBMP("../blue_robot.bmp");
    if (!blueRobot)
        cout << "Unable to load blue robot bmp: " << SDL_GetError() << endl;
    else
        SDL_SetColorKey(blueRobot, SDL_SRCCOLORKEY, SDL_MapRGB(blueRobot->format, 255, 255, 255));

    keepGoing = true;
    while (keepGoing)
    {
        SDL_FillRect(screen, NULL, SDL_MapRGB(screen->format, 0, 200, 0));

        SDL_Rect rect;
        if (ball)
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()), ball->w, ball->h);
            SDL_BlitSurface(ball, NULL, screen, &rect);
        }

        Position pos(0,0);
        if (PredictBallPosition(&pos))
        {
            rect = PosToRect(NormalizePosition(allData->ball->GetPos()));
            SDL_Rect rect2 = PosToRect(NormalizePosition(pos));
            DrawLine(screen, rect.x, rect.y, rect2.x, rect2.y, CreateColor(255,0,0));
        }

        if (blueRobot)
        {
            for (int i=0 ; i < 3 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), blueRobot->w, blueRobot->h);
                SDL_BlitSurface(blueRobot, NULL, screen, &rect);
            }
        }

        if (redRobot)
        {
            for (int i=3 ; i < 6 ; i++)
            {
                rect = PosToRect(NormalizePosition(allData->robots[i]->GetPos()), redRobot->w, redRobot->h);
                SDL_BlitSurface(redRobot, NULL, screen, &rect);
            }
        }

        SDL_Flip(screen);
        usleep(20000);
    }

    if (ball)
        SDL_FreeSurface(ball);
    if (redRobot)
        SDL_FreeSurface(redRobot);
    if (blueRobot)
        SDL_FreeSurface(blueRobot);

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

