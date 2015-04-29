//============================================================================
// Name        : soccer_client.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Client for RTDB which controls team 1, Ansi-style
//============================================================================


#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"

using namespace std;


typedef struct
{
  RoboControl *robo;
  RawBall *ball;
  Referee *ref;
} RoboBall;

typedef void (*PlayFunc)(RoboControl[], RawBall, Referee);


static void BeforeKickOff(RoboControl robots[], RawBall ball, Referee ref);
static void KickOff(RoboControl robots[], RawBall ball, Referee ref);
static void BeforePenalty(RoboControl robots[], RawBall ball, Referee ref);
static void Penalty(RoboControl robots[], RawBall ball, Referee ref);
static void PlayOn(RoboControl robots[], RawBall ball, Referee ref);
static void Pause(RoboControl robots[], RawBall ball, Referee ref);
static void TimeOver(RoboControl robots[], RawBall ball, Referee ref);

static void* GoalKeeper(void* data);
//static void* RedMove(void* data);


const eTeam team = BLUE_TEAM;


int main(void)
{
    //--------------------------------- Init --------------------------------------------------

    const int client_nr = 11;
    int rfcomm_nr[] = {0, 1, 2};
    const PlayFunc playFunctions[] = {NULL, BeforeKickOff, KickOff, BeforePenalty, Penalty, PlayOn, Pause, TimeOver};

    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RoboControl robots[3] = {RoboControl(DBC, rfcomm_nr[0]),
                                 RoboControl(DBC, rfcomm_nr[1]),
                                 RoboControl(DBC, rfcomm_nr[2])};
        RoboControl redRobo(DBC, 3);

        RawBall ball(DBC);
        Referee ref(DBC);
        ref.Init();

        //-------------------------------------- Ende Init ---------------------------------

        //pthread_t thread1;
        //RoboBall roboBall = {redRobo, ball, ref};
        //pthread_create(&thread1, NULL, RedMove, &roboBall);

        while (1)
        {
            ePlayMode mode = ref.GetPlayMode();
            cout << "Mode = " << mode << endl;

            PlayFunc fn = playFunctions[mode];

            if (fn)
                fn(robots, ball, ref);

            while (ref.GetPlayMode() == mode)
                usleep(10000);
        }

    }
    catch (DBError err)
    {
        cout << "Client died on Error: " << err.what() << endl;
    }

    cout << "End" << endl;
    return 0;
}


static void BeforeKickOff(RoboControl robots[], RawBall ball, Referee ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref.GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (side == RIGHT_SIDE)
    {
        robots[0].GotoXY(0.3, 0.5, 120, true);
        robots[1].GotoXY(0.3, 0, 120, true);
        robots[2].GotoXY(0.3, -0.5, 120, true);
    }
    else
    {
        robots[0].GotoXY(-0.3, 0.5, 120, true);
        robots[1].GotoXY(-0.3, 0, 120, true);
        robots[2].GotoXY(-0.3, -0.5, 120, true);
    }

    usleep(5000000);
    ref.SetReady(team);
}

static void KickOff(RoboControl robots[], RawBall ball, Referee ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref.GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (ref.GetSide() == side)
        robots[1].GotoPos(ball.GetPos());
}

static void BeforePenalty(RoboControl robots[], RawBall ball, Referee ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref.GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;
    robots[2].GotoXY(0.3, -0.5);

    cout << "Before penalty side = " << ref.GetSide() << endl;

    if (ref.GetSide() == side)
    {
        robots[0].GotoXY(0.3, 0.5);
        robots[1].GotoXY(0, 0, 120, true);
    }
    else
    {
        robots[0].GotoXY(-1.3, 0);
        robots[1].GotoXY(0.3, 0);
    }
}

static void Penalty(RoboControl robots[], RawBall ball, Referee ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref.GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    //This should not be here, but the side is not given during the "before penalty" part, so we have to do it here.
    BeforePenalty(robots, ball, ref);

    cout << "Penalty side = " << ref.GetSide() << endl;

    if (ref.GetSide() == side)
    {
        robots[1].GotoPos(ball.GetPos());
    }
    else
    {
        RoboBall roboBall = {&(robots[0]), &ball, &ref};
        GoalKeeper(&roboBall);
    }
}

static void PlayOn(RoboControl robots[], RawBall ball, Referee ref)
{
    pthread_t thread1;
    RoboBall roboBall = {&(robots[0]), &ball, &ref};
    pthread_create(&thread1, NULL, GoalKeeper, &roboBall);

    while (ref.GetPlayMode() == PLAY_ON)
    {
        //TODO

        usleep(30000);
    }

    pthread_join(thread1, NULL);
}

static void Pause(RoboControl robots[], RawBall ball, Referee ref)
{
    //Well, nothing to do, just wait...
}

static void TimeOver(RoboControl robots[], RawBall ball, Referee ref)
{
    //Well, nothing to do, just stop.
}


static void* GoalKeeper(void* data)
{
    RoboBall* roboBall = (RoboBall*)data;
    ePlayMode mode;

    while ((mode = roboBall->ref->GetPlayMode()) == PLAY_ON || mode == PENALTY)
    {

        Position bluePos = roboBall->robo->GetPos();
        Position ballPos = roboBall->ball->GetPos();

        double y = ballPos.GetY();

        if (y > 0.15)
            y = 0.15;
        else if (y < -0.15)
            y = -0.15;

        double deltaY = fabs(bluePos.GetY() - y);

        if (deltaY >= 0.05)
        {
            cout << "Goal keeper moving to y = " << y << endl;
            roboBall->robo->GotoXY(-1.300, y, 160 * deltaY / 0.3, false);
        }

        usleep(30000);
    }

    return NULL;
}

/*static void* RedMove(void* data)
{
    RoboBall* roboBall = (RoboBall*)data;

    while (1)
    {
        Position ballPos = roboBall->ball.GetPos();
        roboBall->robo.GotoXY(ballPos.GetX(), ballPos.GetY(), 120, false);
        cout << "Red moving to " << ballPos << endl << endl;
        usleep(2000000);
    }

    return NULL;
}*/
