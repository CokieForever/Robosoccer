/*
 * Old-fashioned FSM.
 * Deprecated, kept for record only!
 *
 */

#include "standardfsm.h"
#include "log.h"

using namespace std;


static eTeam team = BLUE_TEAM; /**< TODO */

static void BeforeKickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void KickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void BeforePenalty(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void Penalty(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void PlayOn(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void Pause(NewRoboControl *robots[], RawBall *ball, Referee *ref);
static void TimeOver(NewRoboControl *robots[], RawBall *ball, Referee *ref);

static void* GoalKeeper(void* data);


/**
 * @brief Main loop of the FSM.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 * @param t Our team.
 */
void StandardFSM(NewRoboControl *robots[], RawBall *ball, Referee *ref, eTeam t)
{
    const PlayFunc playFunctions[] = {NULL, BeforeKickOff, KickOff, BeforePenalty, Penalty, PlayOn, Pause, TimeOver};
    team = t;

    while (1)
    {
        ePlayMode mode = ref->GetPlayMode();
        Log("Mode = " + ToString(mode), INFO);

        PlayFunc fn = playFunctions[mode];

        if (fn)
        {
            Log("Entering Play function", DEBUG);
            fn(robots, ball, ref);
        }

        Log("Left mode function", DEBUG);

        while (ref->GetPlayMode() == mode)
            usleep(10000);
    }
}


/**
 * @brief State of the FSM: "Before kick-off".
 *
 * Moves the robots to the default kick-off positions.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
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

/**
 * @brief State of the FSM: "Kick-off".
 *
 * Pilots the first robot to do the kick-off if this is our turn, or does nothing if not.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
static void KickOff(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (ref->GetSide() == side)
    {
        Position ballPos = ball->GetPos();

        Log("Kick off!", INFO);
        robots[1]->GotoXY(ballPos.GetX(), ballPos.GetY());
    }
}

/**
 * @brief State of the FSM: "Before penalty".
 *
 * Moves the robots to the default positions depending on the team which will do the penalty shooting.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
static void BeforePenalty(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;
    robots[2]->GotoXY(0.3, -0.5);

    Log("Before penalty side = " + ToString(ref->GetSide()), INFO);

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

/**
 * @brief State of the FSM: "Penalty shooting".
 *
 * Pilots the first robot to do a penalty shooting if this is our turn, or pilots the goal keeper if this is not.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
static void Penalty(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    //This should not be here, but the side is not given during the "before penalty" part, so we have to do it here.
    BeforePenalty(robots, ball, ref);
    usleep(5000000);

    Log("Penalty side = " + ToString(ref->GetSide()), INFO);

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

        Log("Target Y = " + ToString(targetPos.GetY()), DEBUG);

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

/**
 * @brief State of the FSM: "Play on".
 *
 * Launches the goal keeper thread. The behavior of other players is not implemented.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
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

/**
 * @brief State of the FSM: "Pause".
 *
 * Stop all robots after executing the last orders.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
static void Pause(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    //Well, nothing to do, just wait...
}

/**
 * @brief State of the FSM: "Time over".
 *
 * Stops all robots after executing the last orders.
 *
 * @param robots[] The six robots.
 * @param ball The ball.
 * @param ref The referee.
 */
static void TimeOver(NewRoboControl *robots[], RawBall *ball, Referee *ref)
{
    //Well, nothing to do, just stop.
}


/**
 * @brief Main loop to pilot the goal keeper, designed to be launched in a separate thread.
 *
 * @param data A pointer to a @ref RoboBall structure containing useful data.
 * @return NULL
 */
static void* GoalKeeper(void* data)
{
    RoboBall* roboBall = (RoboBall*)data;
    ePlayMode mode;

    eSide side = (team == BLUE_TEAM) ^(roboBall->ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    Log("Goal keeper started", INFO);

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

    Log("End of Goal Keeper", INFO);

    return NULL;
}
