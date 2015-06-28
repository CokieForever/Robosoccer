/*
 * Old-fashioned FSM.
 * Deprecated, kept for record only!
 *
 */

#include "standardfsm.h"

using namespace std;


static eTeam team = BLUE_TEAM;

static void BeforeKickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void KickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void BeforePenalty(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void Penalty(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void PlayOn(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void Pause(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void TimeOver(NewRoboControl *robots[], RawBall *ball, Referee *ref);

static void* GoalKeeper(void* data);


void StandardFSM(NewRoboControl *robots[], RawBall *ball, Referee *ref, eTeam t)
{
    const PlayFunc playFunctions[] = {NULL, BeforeKickOff, KickOff, BeforePenalty, Penalty, PlayOn, Pause, TimeOver};
    team = t;

    while (1)
    {
        ePlayMode mode = ref->GetPlayMode();
        cout << "Mode = " << mode << endl;

        PlayFunc fn = playFunctions[mode];

        if (fn)
        {
            cout << "Entering Play function" << endl;
            fn(robots, ball, ref);
        }

        cout << "Left mode function" << endl;

        while (ref->GetPlayMode() == mode)
        usleep(10000);
    }
}


static void BeforeKickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (side == RIGHT_SIDE)
    {
        robots[0]->GotoXY(0.3, 0.5);
        robots[1]->GotoXY(0.3, 0);
        robots[2]->GotoXY(0.3, -0.5);
    }
    else
    {
        robots[0]->GotoXY(-0.3, 0.5);
        robots[1]->GotoXY(-0.3, 0);
        robots[2]->GotoXY(-0.3, -0.5);
    }

    usleep(5000000);
    ref->SetReady(team);
}

static void KickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (ref->GetSide() == side)
    {
        Position ballPos = ball->GetPos();

        cout << "Kick off!" << endl;
        robots[1]->GotoXY(ballPos.GetX(), ballPos.GetY());
    }
}

static void BeforePenalty(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;
    robots[2]->GotoXY(0.3, -0.5);

    cout << "Before penalty side = " << ref->GetSide() << endl;

    if (ref->GetSide() == side)
    {
        robots[0]->GotoXY(0.3, 0.5);
        robots[1]->GotoXY(0, 0);
    }
    else
    {
        robots[0]->GotoXY(-1.3, 0);
        robots[1]->GotoXY(0.3, 0);
    }
}

static void Penalty(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    //This should not be here, but the side is not given during the "before penalty" part, so we have to do it here.
    BeforePenalty(robots, ball, ref);
    usleep(5000000);

    cout << "Penalty side = " << ref->GetSide() << endl;

    if (ref->GetSide() == side)
    {
        /*Position goalKeeperPos = robots[3].GetPos();
        for (int i=4 ; goalKeeperPos.GetX() >= -0.1 && i < 6 ; i++)
            goalKeeperPos = robots[i].GetPos();*/

        Position targetPos(1.367, 0 /*goalKeeperPos.GetY() >= 0 ? -0.10 : 0.10*/);
        Position ballPos = ball->GetPos();

        double deltaD = ballPos.DistanceTo(targetPos);
        double deltaY = targetPos.GetY() - ballPos.GetY();

        double roboY = targetPos.GetY() - (deltaD + 0.15) * deltaY / deltaD;

        cout << "Target Y = " << targetPos.GetY() << endl;

        robots[1]->GotoXY(0, roboY);
        usleep(3000000);
        robots[1]->GotoXY(ballPos.GetX(), roboY, 160, false);
    }
    else
    {
        RoboBall roboBall = {robots[0], ball, ref};
        GoalKeeper(&roboBall);
    }
}

static void PlayOn(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    pthread_t thread1;
    RoboBall roboBall = {robots[0], ball, ref};
    pthread_create(&thread1, NULL, GoalKeeper, &roboBall);

    while (ref->GetPlayMode() == PLAY_ON)
    {
        //TODO

        usleep(30000);
    }

    pthread_join(thread1, NULL);
}

static void Pause(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    //Well, nothing to do, just wait...
}

static void TimeOver(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    //Well, nothing to do, just stop.
}


static void* GoalKeeper(void* data)
{
    RoboBall* roboBall = (RoboBall*)data;
    ePlayMode mode;

    eSide side = (team == BLUE_TEAM) ^(roboBall->ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    cout << "Goal keeper started" << endl;

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
            roboBall->robo->GotoXY(side == LEFT_SIDE ? -1.300 : +1.300, y, 160 * deltaY / 0.3, false);
        }

        usleep(30000);
    }

    cout << "End of Goal Keeper" << endl;

    return NULL;
}
