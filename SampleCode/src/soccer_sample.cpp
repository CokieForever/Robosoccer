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
#include "coordinates.h"
#include "ballmonitor.h"
#include "refereedisplay.h"
#include "robotmonitor.h"

using namespace std;


typedef struct
{
  RoboControl *robo;
  RawBall *ball;
  Referee *ref;
} RoboBall;

typedef void (*PlayFunc)(RoboControl**, RawBall*, Referee*);


static void BeforeKickOff(RoboControl *robots[], RawBall *ball, Referee *ref);
static void KickOff(RoboControl *robots[], RawBall *ball, Referee *ref);
static void BeforePenalty(RoboControl *robots[], RawBall *ball, Referee *ref);
static void Penalty(RoboControl *robots[], RawBall *ball, Referee *ref);
static void PlayOn(RoboControl *robots[], RawBall *ball, Referee *ref);
static void Pause(RoboControl *robots[], RawBall *ball, Referee *ref);
static void TimeOver(RoboControl *robots[], RawBall *ball, Referee *ref);

static void* GoalKeeper(void* data);

const eTeam team = BLUE_TEAM;


int main(void)
{
    //--------------------------------- Init -------------------- Can't connect RF------------------------------

    const int client_nr = 13;
    int rfcomm_nr_blue[] = {0, 1, 2};
    int rfcomm_nr_red[] = {3, 4, 5};
    const PlayFunc playFunctions[] = {NULL, BeforeKickOff, KickOff, BeforePenalty, Penalty, PlayOn, Pause, TimeOver};

    int *rfcomm_nr = team == BLUE_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;
    int *rfcomm_nr_2 = team == RED_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;

    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RoboControl robo1 = RoboControl(DBC, rfcomm_nr[0]);
        RoboControl robo2 = RoboControl(DBC, rfcomm_nr[1]);
        RoboControl robo3 = RoboControl(DBC, rfcomm_nr[2]);
        RoboControl robo4 = RoboControl(DBC, rfcomm_nr_2[0]);
        RoboControl robo5 = RoboControl(DBC, rfcomm_nr_2[1]);
        RoboControl robo6 = RoboControl(DBC, rfcomm_nr_2[2]);

        RoboControl *robots[] = {&robo1, &robo2, &robo3, &robo4, &robo5, &robo6};

        RawBall ball(DBC);
        Referee ref(DBC);
        ref.Init();

        //SetManualCoordCalibration(Position(0,-0.867), Position(1.367,0), Position(0,0.867), Position(-1.367,0));
        SetManualCoordCalibration(Position(-0.03,-0.826), Position(1.395,0.08), Position(-0.027,0.908), Position(-1.44,0.036));
        StartBallMonitoring(&ball);

        StartRefereeDisplay(robots, &ball, team);

        SetAllRobots(robots);

        //-------------------------------------- Ende Init ---------------------------------

        while (1)
        {
            ePlayMode mode = ref.GetPlayMode();
            cout << "Mode = " << mode << endl;

            PlayFunc fn = playFunctions[mode];

            if (fn)
            {
                cout << "Entering Play function" << endl;
                fn(robots, &ball, &ref);
            }

            cout << "Left mode function" << endl;

            while (ref.GetPlayMode() == mode)
                usleep(10000);
        }

    }
    catch (DBError err)
    {
        cout << "Client died on Error: " << err.what() << endl;
    }

    StopRefereeDisplay();
    StopBallMonitoring();

    cout << "End" << endl;
    return 0;
}


static void BeforeKickOff(RoboControl *robots[], RawBall *ball, Referee *ref)
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

static void KickOff(RoboControl *robots[], RawBall *ball, Referee *ref)
{
    eSide side = (team == BLUE_TEAM) ^(ref->GetBlueSide() == LEFT_SIDE) ? RIGHT_SIDE : LEFT_SIDE;

    if (ref->GetSide() == side)
    {
        Position ballPos = ball->GetPos();

        cout << "Kick off!" << endl;
        robots[1]->GotoXY(ballPos.GetX(), ballPos.GetY());
    }
}

static void BeforePenalty(RoboControl *robots[], RawBall *ball, Referee *ref)
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

static void Penalty(RoboControl *robots[], RawBall *ball, Referee *ref)
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

static void PlayOn(RoboControl *robots[], RawBall *ball, Referee *ref)
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

static void Pause(RoboControl *robots[], RawBall *ball, Referee *ref)
{
    //Well, nothing to do, just wait...
}

static void TimeOver(RoboControl *robots[], RawBall *ball, Referee *ref)
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

int Milestone2Part2(void)
{
	//--------------------------------- Init --------------------------------------------------

	/** Use client number according to your account number!
	 *
	 *	This is necessary in order to assure that there are unique
	 *	connections to the RTDB.
	 *
	 */
        const int client_nr = 15;

	/** Type in the rfcomm number of the robot you want to connect to.
	 *  The numbers of the robots you are connected to can be found on the
	 *  screen when you connected to them.
	 *
	 *  The red robots' number will be in the range of 3 to 5
	 *  The blue robots' number will be in the range from 0 to 2
	 *
	 *  The robots are always connected to the lowest free rfcomm device.
	 *  Therefore if you have two blue robots connected the will be
	 *  connected to rfcomm number 0 and number 1...
	 *
	 */
        int rfcomm_nr = 0;

	try {

		/** Establish connection to the RTDB.
		 *
		 *  The connection to the RTDB is necessary in order to get access
		 *  to the control and the status of the robots which are both stored
		 *  in the RTDB.
		 *
		 *  In the RTDB there are also informations about the ball and the
		 *  other robot positions.
		 *
		 */
		cout << endl << "Connecting to RTDB..." << endl;
		/** Create the client name with the unique client number*/
		string client_name = "pololu_client_";
		client_name.push_back((char) (client_nr + '0'));
		RTDBConn DBC(client_name.data(), 0.1, "");

		/** Create a new RoboControl object.
		 *
		 *  This is the basis for any communication with the robot.
		 *
		 *  We need to hand over the RTDB connection (DBC) and the rfcomm
		 *  number of the robot we want to control.
		 */
		RoboControl robo(DBC, rfcomm_nr);

		/** Now let's print out some information about the robot... */

                cout << "Robo @ rfcomm" << rfcomm_nr << endl;
                cout << "\t initial position: " << robo.GetPos() << endl;
		cout << "\t initial rotation: " << robo.GetPhi() << endl;

		/** Create a ball object
		 *
		 *  This ball abject gives you access to all information about the ball
		 *  which is extracted from the cam.
		 *
		 */
		RawBall ball(DBC);
		/** lets print this information: */
		cout << "Ball informations:" << endl;
		cout << "\t initial position: " << ball.GetPos() << endl;
		/** Notice that the rotation here refers to the moving direction of the ball.
		 *  Therefore if the ball does not move the rotation is not defined.
		 */
		cout << "\t initial direction: " << ball.GetPhi() << endl;
		cout << "\t initial velocity: " << ball.GetVelocity() << endl;

		//-------------------------------------- Ende Init ---------------------------------


                double a;
                double b;
                double x1;
                double x2;
                cout <<  "initial ball pos " << ball.GetPos() << endl ;
                while (1)
                {

                    bool test=  ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )) &&  !(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)));

                    if (test)

                    {
                        cout <<  "Moving to " << ball.GetPos() << endl ;
                        a = (ball.GetY()-robo.GetY())/(ball.GetX()-robo.GetX());
                        b=ball.GetY()-a*ball.GetX();
                        x1= (0.42 -b)/a;
                        x2= (-0.34-b)/a;
                        if (((fabs(x1)>1.14) && (fabs(x1)<1.5) && !((robo.GetY()>0.37 && ball.GetY()>0.37) || (robo.GetY()<0.37 && ball.GetY()<0.37)))
                                || ((fabs(x2)>1.26)&& (fabs(x2)<1.5)  && !((robo.GetY()>-0.34 && ball.GetY()>-0.34) || (robo.GetY()<-0.34 && ball.GetY()<-0.34))))
                        {
                            cout <<  "intersection " << ball.GetPos() << endl ;
                            if ((robo.GetX()>0) && (ball.GetX()>0) && ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )) &&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                            {
                                cout << "same positive signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2 <<endl ;
                                Position pos5(1.0, robo.GetY());
                                cout << "step1" << endl;
                                robo.GotoXY(pos5.GetX(),pos5.GetY());
                                while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                     {cout << "zzzz" << endl;
                                    Position pos5(1.0, robo.GetY());
                                    robo.GotoXY(pos5.GetX(),pos5.GetY());
                                    usleep(50000);}
                                cout << "end step1" << endl;
                                Position pos6(1.0, ball.GetY());
                                cout << "step2" << endl;
                                robo.GotoXY(pos6.GetX(),pos6.GetY());
                                while (robo.GetPos().DistanceTo(pos6) > 0.10 &&(( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))

                                     {cout << "zzzz" << endl;
                                    Position pos6(1.0, ball.GetY());
                                    robo.GotoXY(pos6.GetX(),pos6.GetY());
                                    usleep(50000);}
                                cout << "end step2" << endl;
                                robo.GotoXY(ball.GetX(),ball.GetY(),160,true);

                            }
                            else if ((robo.GetX()<0) && (ball.GetX()<0) && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                            {
                                cout << "same negative signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2<< endl ;
                                Position pos5(-1.0, robo.GetY());
                                cout << "step1" << endl;
                                robo.GotoXY(pos5.GetX(),pos5.GetY());
                                while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                 {cout << "zzzz" << endl;
                                    Position pos5(-1.0, robo.GetY());
                                    robo.GotoXY(pos5.GetX(),pos5.GetY());
                                    usleep(50000);}
                                Position pos6(-1.0, ball.GetY());
                                cout << "step2" << endl;
                                robo.GotoXY(pos6.GetX(),pos6.GetY());
                                while (robo.GetPos().DistanceTo(pos6) > 0.10 && ( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                                     {cout << "zzzz" << endl;
                                    Position pos6(-1.0, ball.GetY());
                                    robo.GotoXY(pos6.GetX(),pos6.GetY());
                                    usleep(50000);}
                                robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                            }
                            else
                            {
                                cout << "different signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2<< endl ;
                                if (( ((fabs(ball.GetX())<1.4) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                                {
                                    Position pos5(0.0, robo.GetY());
                                    robo.GotoXY(pos5.GetX(),pos5.GetY());
                                    while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                         {cout << "zzzz" << endl;
                                    usleep(50000);}
                                    Position pos6(0.0, ball.GetY());
                                    robo.GotoXY(pos6.GetX(),pos6.GetY());
                                    while (robo.GetPos().DistanceTo(pos6) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                         {cout << "zzzz" << endl;
                                    usleep(50000);}
                                    robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                                }
                            }

                        }
                        else
                        {
                            cout <<  "pas dintersection " << ball.GetPos() << x1<< "   "<< x2<< endl ;
                            if (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                            {
                                robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                                //while (robo.GetPos().DistanceTo(ball.GetPos()) > 0.10 && (((fabs(ball.GetX())<1.3) && (fabs(ball.GetY())<0.75 )&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))) )
                                //{
                                   // usleep(30000);



                                //}

                            }
                        }

                    }
                    else if ((fabs(ball.GetX())<1.127) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()>0 ))
                    {

                            cout <<  "Moving to " << ball.GetPos() << endl ;
                            a = (ball.GetY()-robo.GetY())/(ball.GetX()-robo.GetX());
                            b=ball.GetY()-a*ball.GetX();
                            x1= (0.42 -b)/a;
                            x2= (-0.34-b)/a;
                            if (((fabs(x1)>1.14) && !((robo.GetY()>0.37 && ball.GetY()>0.37) || (robo.GetY()<0.37 && ball.GetY()<0.37)))
                                    || ((fabs(x2)>1.26)  && !((robo.GetY()>-0.34 && ball.GetY()>-0.34) || (robo.GetY()<-0.34 && ball.GetY()<-0.34))))
                            {
                                cout <<  "intersection " << ball.GetPos() << endl ;
                                if ((robo.GetX()>0) && (ball.GetX()>0) && ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )) &&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                                {
                                    cout << "same positive signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2 <<endl ;
                                    Position pos5(1.0, robo.GetY());
                                    cout << "step1" << endl;
                                    robo.GotoXY(pos5.GetX(),pos5.GetY());
                                    while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                         {cout << "zzzz" << endl;
                                        Position pos5(1.0, robo.GetY());
                                        robo.GotoXY(pos5.GetX(),pos5.GetY());
                                        usleep(50000);}
                                    cout << "end step1" << endl;
                                    Position pos6(1.0, ball.GetY());
                                    cout << "step2" << endl;
                                    robo.GotoXY(pos6.GetX(),pos6.GetY());
                                    while (robo.GetPos().DistanceTo(pos6) > 0.10 &&(( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))

                                         {cout << "zzzz" << endl;
                                        Position pos6(1.0, ball.GetY());
                                        robo.GotoXY(pos6.GetX(),pos6.GetY());
                                        usleep(50000);}
                                    cout << "end step2" << endl;
                                    robo.GotoXY(ball.GetX(),ball.GetY(),160,true);

                                }
                                else if ((robo.GetX()<0) && (ball.GetX()<0) && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                {
                                    cout << "same negative signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2<< endl ;
                                    Position pos5(-1.0, robo.GetY());
                                    cout << "step1" << endl;
                                    robo.GotoXY(pos5.GetX(),pos5.GetY());
                                    while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                     {cout << "zzzz" << endl;
                                        Position pos5(-1.0, robo.GetY());
                                        robo.GotoXY(pos5.GetX(),pos5.GetY());
                                        usleep(50000);}
                                    Position pos6(-1.0, ball.GetY());
                                    cout << "step2" << endl;
                                    robo.GotoXY(pos6.GetX(),pos6.GetY());
                                    while (robo.GetPos().DistanceTo(pos6) > 0.10 && ( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                                         {cout << "zzzz" << endl;
                                        Position pos6(-1.0, ball.GetY());
                                        robo.GotoXY(pos6.GetX(),pos6.GetY());
                                        usleep(50000);}
                                    robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                                }
                                else
                                {
                                    cout << "different signs" << robo.GetPos()<< "    " << ball.GetPos() << x1<< "   "<< x2<< endl ;
                                    if (( ((fabs(ball.GetX())<1.4) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0))))
                                    {
                                        Position pos5(0.0, robo.GetY());
                                        robo.GotoXY(pos5.GetX(),pos5.GetY());
                                        while (robo.GetPos().DistanceTo(pos5) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                             {cout << "zzzz" << endl;
                                        usleep(50000);}
                                        Position pos6(0.0, ball.GetY());
                                        robo.GotoXY(pos6.GetX(),pos6.GetY());
                                        while (robo.GetPos().DistanceTo(pos6) > 0.10 && (( ((fabs(ball.GetX())<1.417) && (fabs(ball.GetY())<0.82 )))&&!(((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.4) )||((fabs(ball.GetX())>1.14) && (fabs(ball.GetY())<0.45) && (ball.GetY()>0.0) && (ball.GetX()>0.0)))))
                                             {cout << "zzzz" << endl;
                                        usleep(50000);}
                                        robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                                    }
                                }

                            }
                        else {
                        cout <<"ball near the wall no intersection" << ball.GetPos() << endl;
                        Position pos5(robo.GetX(), 0.6);
                        robo.GotoXY(pos5.GetX(),pos5.GetY());
                        while (robo.GetPos().DistanceTo(pos5) > 0.10 && ((fabs(ball.GetX())<1.127) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()>0 )))
                             {cout << "zzzz" << endl;
                                    usleep(50000);}
                        Position pos6(ball.GetX(), 0.6);
                        robo.GotoXY(pos6.GetX(),pos6.GetY());
                        while (robo.GetPos().DistanceTo(pos6) > 0.10 && ((fabs(ball.GetX())<1.127) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()>0 )))
                             {cout << "zzzz" << endl;
                                    usleep(50000);}
                        robo.GotoXY(ball.GetX(),ball.GetY(),160,true);

                        }
                    }
                    else if ((fabs(ball.GetX())<1.1) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()<0 ))
                    {
                        cout <<"ball near the wall " << ball.GetPos() << endl;
                        Position pos5(robo.GetX(), -0.6);
                        robo.GotoXY(pos5.GetX(),pos5.GetY());
                        while (robo.GetPos().DistanceTo(pos5) > 0.10 && ((fabs(ball.GetX())<1.1) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()>0 )))
                             {cout << "zzzz" << endl;
                                    usleep(50000);}
                        Position pos6(ball.GetX(), -0.6);
                        robo.GotoXY(pos6.GetX(),pos6.GetY());
                        while (robo.GetPos().DistanceTo(pos6) > 0.10 && ((fabs(ball.GetX())<1.1) && (fabs(ball.GetY())<0.9 ) && (ball.GetY()>0 )))
                             {cout << "zzzz" << endl;
                                    usleep(50000);}
                        robo.GotoXY(ball.GetX(),ball.GetY(),160,true);
                    }
                    else

                        cout <<"ball in forbidden region " << ball.GetPos() << endl;

		}

        }


        catch (DBError err)
        {
		cout << "Client died on Error: " << err.what() << endl;
	}

        cout << "End" << endl;
	return 0;
}
