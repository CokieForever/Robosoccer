//============================================================================
// Name        : soccer_client.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Client for RTDB which controls team 1, Ansi-style
//============================================================================


#include <time.h>
#include <iostream>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include <stdlib.h>
#include <pthread.h>
#include "math.h"


using namespace std;

Position posdia1(-0.7,-0.7);
Position posdia2(0.7,0.7);
Position posdia3(0.7,-0.7);
Position posdia4(-0.7,0.7);

static RoboControl *robo_global1 = NULL;
static RoboControl *robo_global2 = NULL;
static RoboControl *robo_global3 = NULL;
static RoboControl *robo_global4 = NULL;

int checkdirec;//check if the robot will turn right or left 1/-1; very important for avoiding random walking zombie..
int speed=100;//the speed of robots cruising diagonally
int speed2=40;//the speed of random walking robot
double threshold=0.30;//if the perpendicular distance between obstacle and the track of robot smaller than threshold, the obstacle is considered as dangerous
double collision_distance=1.0;//the robot only considers obstacle within this distance from itself.
int turn_direction=1;//the direction of turning of the robot, 1 clockwise, -1 anti-clockwise.
bool random_walking_warning;//check if the obstacle is random walking zombie..

Position same(Position target);
//this same function is not important, just used to test if we can change the position through functions;

double pi=3.14;
double angle=50;//in case of obstacle present, the robot will rotate this angle
//the following are the coefficience of rotation matrix
double a11 = cos(angle*pi/180.0);
double a12 = -sin(angle*pi/180.0);
double a21 = sin(angle*pi/180.0);
double a22 = cos(angle*pi/180.0);
bool keepGoing;//not important, just for test.
//the following is not important, just for test
void* ThreadFunction(void *data)
{
    RoboControl *robo1 = (RoboControl*)data;

    keepGoing = true;
    while (keepGoing)
    {
        cout << "position of robo1: " << robo1->GetPos() << endl;
        usleep(2e6);
    }

    return NULL;
}
//the following is the collision avoidance function.
//robo is the robot that you want to control.
//obstacle1~3 are obstacle robots that are taken into consideration
//destination
//priority is used so that robots with higher priority can move first, while the robot of lower priority waits.
void avoidance(RoboControl *robo, RoboControl *obstacle1, RoboControl *obstacle2, RoboControl *obstacle3, Position destination, int priority)
{
    double x_rbt,y_rbt,x_ob1,y_ob1, x_ob2,y_ob2, x_ob3,y_ob3, x0,y0;//coordination of the robot and obstacle and destination
    double direx, direy, distance,ob_distance1, ob_distance2, ob_distance3, xtempo, ytempo;
    //direx, direy are the vector that shows the current moving direction of the robot.
    //ob_distance1~3 are the distance between the robot and the obstacles
    //xtempo, ytempo show the temporary destination of the robot(in the case of robot turning and avoding collision)
    double avoidx, avoidy;
    //avoidx, avoidy, the vector from current position of the robot to the place where the robot will go after turning, in order to avoid collision;
    double a,b,c,d1, d2, d3;//a, b and c are parameters of the line between the robot and the destination
    //d1, d2, d3 are the perpendicular distance between obstacles and the track of the robot
    int seen1, seen2, seen3, total;
    //if obstacle is in the area of forward direction of the robot, 1 for yes, 0 for no and the totol obstacle on the track
    int waiting;
    //waiting time of the robot with lower priority
    bool collision; //if yes 1, no 0;
    //robo->GotoXY(destination.GetX(), destination.GetY(), speed, true);
    //the following are used to initiallize the beginning condition
    x0=destination.GetX();
    y0=destination.GetY();
    xtempo=x0;
    ytempo=y0;
    collision=1;
    while (robo->GetPos().DistanceTo(destination) > 0.10)
        //usleep(50000);
        {
            //while(collision)
            //{
                //following is to change the destination to temporary destination, in order to avoid collision
                x0=xtempo;
                y0=ytempo;
                //coordinate of robot
                x_rbt=robo->GetX();
                y_rbt=robo->GetY();
                //coordinate of obstacles
                x_ob1=obstacle1->GetX();
                y_ob1=obstacle1->GetY();
                x_ob2=obstacle2->GetX();
                y_ob2=obstacle2->GetY();
                x_ob3=obstacle3->GetX();
                y_ob3=obstacle3->GetY();
                //distance between robot and (temporary) destination
                distance=sqrt((x0-x_rbt)*(x0-x_rbt)+(y0-y_rbt)*(y0-y_rbt));
                //distance between the robot and obstacles
                ob_distance1=(robo->GetPos().DistanceTo(obstacle1->GetPos()));
                ob_distance2=(robo->GetPos().DistanceTo(obstacle2->GetPos()));
                ob_distance3=(robo->GetPos().DistanceTo(obstacle3->GetPos()));
                //direction from robot to destination
                direx=(x0-x_rbt)/distance;
                direy=(y0-y_rbt)/distance;
                    //area1x=direy;
                    //area1y=-direx;
                    //area2x=-area1x;
                    //area2y=-area1y;
                //calculate the parameter of the line between (temporary) destination and robot with the form ax+by+c=0
                a=y_rbt-y0;
                b=x0-x_rbt;
                c=(y0-y_rbt)*x_rbt-(x0-x_rbt)*y_rbt;
                //calculate the perpendicular distance between obstacles to the line between (temporary) destination and the robot.
                d1=abs(a*x_ob1+b*y_ob1+c)/sqrt(a*a+b*b);
                d2=abs(a*x_ob2+b*y_ob2+c)/sqrt(a*a+b*b);
                d3=abs(a*x_ob3+b*y_ob3+c)/sqrt(a*a+b*b);
                seen1=1;
                //check if obstacle1 is on the way of the track
                if(((x_ob1>x_rbt)&(x_ob1>x0))|((x_ob1<x_rbt)&(x_ob1<x0)))
                {
                    if(~((((y_rbt<y_ob1)&(y_ob1<y0))|((y_rbt>y_ob1)&(y_ob1>y0)))&(d1<threshold)))
                    {
                        seen1=0;
                    }
                }
                if(((y_ob1>y_rbt)&(y_ob1>y0))|((y_ob1<y_rbt)&(y_ob1<y0)))
                {
                    if(~((((x_rbt<x_ob1)&(x_ob1<x0))|((x_rbt>x_ob1)&(x_ob1>x0)))&(d1<threshold)))
                    {
                        seen1=0;
                    }
                }
                seen2=1;
                //check if obstacle2 is on the way of the track
                if(((x_ob2>x_rbt)&(x_ob2>x0))|((x_ob2<x_rbt)&(x_ob2<x0)))
                {
                    if(~((((y_rbt<y_ob2)&(y_ob2<y0))|((y_rbt>y_ob1)&(y_ob2>y0)))&(d2<threshold)))
                    {
                        seen2=0;
                    }
                }
                if(((y_ob2>y_rbt)&(y_ob2>y0))|((y_ob2<y_rbt)&(y_ob2<y0)))
                {
                    if(~((((x_rbt<x_ob2)&(x_ob2<x0))|((x_rbt>x_ob2)&(x_ob2>x0)))&(d2<threshold)))
                    {
                        seen2=0;
                    }
                }
                seen3=1;
                //check if obstacle3 is on the way of the track
                if(((x_ob3>x_rbt)&(x_ob3>x0))|((x_ob3<x_rbt)&(x_ob3<x0)))
                {
                    if(~((((y_rbt<y_ob3)&(y_ob3<y0))|((y_rbt>y_ob3)&(y_ob3>y0)))&(d3<threshold)))
                    {
                        seen3=0;
                    }
                }
                if(((y_ob3>y_rbt)&(y_ob3>y0))|((y_ob3<y_rbt)&(y_ob3<y0)))
                {
                    if(~((((x_rbt<x_ob3)&(x_ob3<x0))|((x_rbt>x_ob3)&(x_ob3>x0)))&(d3<threshold)))
                    {
                        seen3=0;
                    }
                }
                collision=0;
                //if((d<threshold)&seen1&(~(seen2&(d2<threshold))))
                /*
                if(((d1<threshold)&seen1)&(seen2&(d2<threshold)))
                {
                    collision=1;
                    if(((y_rbt-ytempo)*(x_ob1-x_rbt)+(xtempo-x_rbt)*(y_ob1-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(0*direx-1*direy)*ob_distance1;
                        avoidy=(1*direx+0*direx)*ob_distance1;
                        checkdirec=1;
                        collision=0;
                    }
                    else
                    {
                        avoidx=(0*direx+1*direy)*ob_distance1;
                        avoidy=(-1*direx+0*direx)*ob_distance1;
                        checkdirec=0;
                        collision=0;
                    }
                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob1-x_rbt)<<", y="<<(y_ob1-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                else if((d1<threshold)&seen1)
                {
                    collision=1;
                    if(((y_rbt-ytempo)*(x_ob1-x_rbt)+(xtempo-x_rbt)*(y_ob1-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(0.939*direx-0.342*direy)*ob_distance1;
                        avoidy=(0.342*direx+0.939*direx)*ob_distance1;
                        checkdirec=1;
                    }
                    else
                    {
                        avoidx=(0.939*direx+0.342*direy)*ob_distance1;
                        avoidy=(-0.342*direx+0.939*direx)*ob_distance1;
                        checkdirec=0;
                    }
                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob1-x_rbt)<<", y="<<(y_ob1-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                //if((~((d<threshold)&seen1))&(seen2&(d2<threshold)))
                else if(seen2&(d2<threshold))
                {
                    collision=1;
                    if(((y_rbt-ytempo)*(x_ob2-x_rbt)+(xtempo-x_rbt)*(y_ob2-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(0.939*direx-0.342*direy)*ob_distance2;
                        avoidy=(0.342*direx+0.939*direx)*ob_distance2;
                        checkdirec=1;
                    }
                    else
                    {
                        avoidx=(0.939*direx+0.342*direy)*ob_distance1;
                        avoidy=(-0.342*direx+0.939*direx)*ob_distance1;
                        checkdirec=0;
                    }
                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob2-x_rbt)<<", y="<<(y_ob2-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                else
                {
                    collision=collision;
                }
                */
                random_walking_warning=0;
                //the following is to check which obstacle is the nearest and therefore try to avoid it.
                if((((((ob_distance1<ob_distance2)&(ob_distance1<ob_distance3))&seen1)|(seen1&(~seen2)&(ob_distance1>ob_distance2)&(ob_distance1<ob_distance3))|(seen1&(~seen3)&(ob_distance1>ob_distance3)&(ob_distance1<ob_distance2)))&(robo->GetPos().DistanceTo(obstacle1->GetPos())<collision_distance)))
                //if((((((d1<d2)&(d1<d3))&seen1)|(seen1&(~seen2)&(d1>d2)&(d1<d3))|(seen1&(~seen3)&(d1>d3)&(d1<d2)))&(robo->GetPos().DistanceTo(obstacle1->GetPos())<collision_distance)))
                {
                    collision=1;
                    /*
                    if(robo->GetPos().DistanceTo(obstacle1->GetPos())<0.3)
                    {
                        avoidx=(0*direx-1*direy)*ob_distance1;
                        avoidy=(1*direx+0*direx)*ob_distance1;
                        collision=0;
                    }
                    */
                    //if there is a potential collision, turn the angle defined before;
                    //in case of avoiding our own robots, all our our own robots will turn right so that they will not run into each other
                    //therefore if-else structure doesn't play a role here
                    //but for random walking zombie, the robot will also turn in the direction which will bring us benefit..
                    if(((y_rbt-ytempo)*(x_ob1-x_rbt)+(xtempo-x_rbt)*(y_ob1-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(a11*direx+a12*direy)*ob_distance1;
                        avoidy=(a21*direx+a22*direy)*ob_distance1;
                        checkdirec=1;
                        //collision=0;
                    }
                    else
                    {
                        avoidx=(a11*direx+a12*direy)*ob_distance1;
                        avoidy=(a21*direx+a22*direy)*ob_distance1;
                        checkdirec=-1;
                        //collision=0;
                    }

                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob1-x_rbt)<<", y="<<(y_ob1-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                if ((((((ob_distance2<ob_distance1)&(ob_distance2<ob_distance3))&seen2)|(seen2&(~seen1)&(ob_distance2>ob_distance1)&(ob_distance2<ob_distance3))|(seen2&(~seen3)&(ob_distance2>ob_distance3)&(ob_distance2<ob_distance1)))&(robo->GetPos().DistanceTo(obstacle2->GetPos())<collision_distance)))
                //else if (((((d2<d1)&(d2<d3))&seen2)|(seen2&(~seen1)&(d2>d1)&(d2<d3))|(seen2&(~seen3)&(d2>d3)&(d2<d1)))&(robo->GetPos().DistanceTo(obstacle2->GetPos())<collision_distance))
                {
                    collision=1;
                    /*
                    if(robo->GetPos().DistanceTo(obstacle2->GetPos())<0.3)
                    {
                        avoidx=(0*direx-1*direy)*ob_distance2;
                        avoidy=(1*direx+0*direx)*ob_distance2;
                        collision=0;
                    }
                    */
                    //if there is a potential collision, turn the angle defined before;
                    //in case of avoiding our own robots, all our our own robots will turn right so that they will not run into each other
                    //therefore if-else structure doesn't play a role here
                    //but for random walking zombie, the robot will also turn in the direction which will bring us benefit..
                    if(((y_rbt-ytempo)*(x_ob2-x_rbt)+(xtempo-x_rbt)*(y_ob2-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(a11*direx+a12*direy)*ob_distance2;
                        avoidy=(a21*direx+a22*direy)*ob_distance2;
                        checkdirec=1;
                        //collision=0;
                    }
                    else
                    {
                        avoidx=(a11*direx+a12*direy)*ob_distance2;
                        avoidy=(a21*direx+a22*direy)*ob_distance2;
                        checkdirec=-1;
                        //collision=0;
                    }
                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob1-x_rbt)<<", y="<<(y_ob1-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                if((((((ob_distance3<ob_distance2)&(ob_distance3<ob_distance1))&seen3)|(seen3&(~seen2)&(ob_distance3>ob_distance2)&(ob_distance3<ob_distance1))|(seen3&(~seen1)&(ob_distance3>ob_distance1)&(ob_distance3<ob_distance2)))&(robo->GetPos().DistanceTo(obstacle3->GetPos())<collision_distance)))
                //else if((((((d3<d2)&(d3<d1))&seen3)|(seen3&(~seen2)&(d3>d2)&(d3<d1))|(seen3&(~seen1)&(d3>d1)&(d3<d2)))&(robo->GetPos().DistanceTo(obstacle3->GetPos())<collision_distance))|(robo->GetPos().DistanceTo(obstacle3->GetPos())<0.3))
                //else if (robo->GetPos().DistanceTo(obstacle3->GetPos())<0.3)
                {
                    collision=1;
                    random_walking_warning=1;
                    /*
                    if(robo->GetPos().DistanceTo(obstacle3->GetPos())<0.3)
                    {
                        avoidx=(0*direx-1*direy)*ob_distance3;
                        avoidy=(1*direx+0*direx)*ob_distance3;
                        collision=0;
                    }
                    */
                    //if there is a potential collision, turn the angle defined before;
                    //in case of avoiding our own robots, all our our own robots will turn right so that they will not run into each other
                    //therefore if-else structure doesn't play a role here
                    //but for random walking zombie, the robot will also turn in the direction which will bring us benefit..
                    if(((y_rbt-ytempo)*(x_ob3-x_rbt)+(xtempo-x_rbt)*(y_ob3-y_rbt))<0)//choose the direction to avoid collision
                    {
                        avoidx=(a11*direx+a12*direy)*0.3;//ob_distance3;
                        avoidy=(a21*direx+a22*direy)*0.3;//ob_distance3;
                        //avoidx=(y_ob3-y_rbt)*0.3;
                        //avoidy=-(x_ob3-x_rbt)*0.3;
                        checkdirec=1;
                        //collision=0;
                    }
                    else
                    {
                        //avoidx=-(y_ob3-y_rbt)*0.3;
                        //avoidy=(x_ob3-x_rbt)*0.3;
                        avoidx=(a11*direx-a12*direy)*0.3;//ob_distance3;
                        avoidy=(-a21*direx+a22*direy)*0.3;//ob_distance3;
                        checkdirec=-1;
                        //collision=0;
                    }
                    cout<<"vector from robot to des is x="<<(xtempo-x_rbt)<<", y="<<(ytempo-y_rbt)<<endl<<endl;
                    cout<<"vector from robot to obstacle is x="<<(x_ob1-x_rbt)<<", y="<<(y_ob1-y_rbt)<<endl<<endl;
                    xtempo=robo->GetX()+avoidx;
                    ytempo=robo->GetX()+avoidy;
                }
                total=seen1+seen2+seen3;//calculate total obstacle that can be seen.
                //following is a priority function, so that robots with lower priority will wait and let robots with higher priority go first.
                if(priority==1)
                    waiting=0;
                else if (priority==2)
                    waiting=600000;
                else if (priority==3)
                    waiting=2000000;
                else
                    waiting=0;
                if(total>1)
                {
                    robo->StopAction();
                    usleep(waiting);
                }
                //above is a priority function
                x0=xtempo;
                y0=ytempo;
                //so that the robot will rotate based on previous angle(temporary destination), not based on the angble to the final destination.
                //aovid collision version1
                /*
                if(collision==0)
                {
                       cout<<"current destination is x="<<xtempo<<", y="<<ytempo<<" ,check is "<<checkdirec<<endl<<endl;
                       robo->GotoXY(xtempo, ytempo, speed, true);robo->
                       usleep(200000);
                       xtempo=destination.GetX();
                       ytempo=destination.GetY();
                       cout<<"current destination is x="<<xtempo<<", y="<<ytempo<<" ,check is "<<checkdirec<<endl<<endl;
                       robo->GotoXY(xtempo, ytempo, speed, true);
                       usleep(100000);
                }
                */
                //avoid collision version2
                if (collision==1)
                {
                    //version1 of turning to avoid collision, by telling the robot to a certain place
                    /*
                    cout<<"current destination is x="<<xtempo<<", y="<<ytempo<<" ,check is "<<checkdirec<<endl<<endl;
                    robo->GotoXY(xtempo, ytempo, speed, true);
                    usleep(30000);
                    */
                    //version2 of turning to avoid collision, by telling the robot to turn at a certain angle
                    if(random_walking_warning)
                    {
                        robo->Turn(angle*turn_direction*checkdirec*(-1));
                    }
                    else
                    {
                        robo->Turn(angle*turn_direction);
                    }
                    //2 versions remain to be tested in real field
                }
                if(collision==0)
                {
                    cout<<"current destination is x="<<xtempo<<", y="<<ytempo<<" ,check is "<<checkdirec<<endl<<endl;
                    robo->GotoXY(xtempo, ytempo, speed, true);
                    usleep(500000);
                    xtempo=destination.GetX();
                    ytempo=destination.GetY();
                    cout<<"current destination is x="<<xtempo<<", y="<<ytempo<<" ,check is "<<checkdirec<<endl<<endl;
                    robo->GotoXY(xtempo, ytempo, speed, true);
                    usleep(100000);
                }

            //}

        }
}

void* ThreadFunction1(void *data)
{
    while(1)
    {
        avoidance(robo_global1, robo_global2, robo_global3, robo_global4, posdia1,1);
        avoidance(robo_global1, robo_global2, robo_global3, robo_global4, posdia2,1);
    }
    return NULL;
}

void* ThreadFunction2(void *data)
{
    while(1)
    {
        avoidance(robo_global2, robo_global1, robo_global3, robo_global4, posdia1,2);
        avoidance(robo_global2, robo_global1, robo_global3, robo_global4, posdia2,2);
    }
    return NULL;
}

void* ThreadFunction3(void *data)
{
    while(1)
    {
        avoidance(robo_global3, robo_global1, robo_global2, robo_global4, posdia3,3);
        avoidance(robo_global3, robo_global1, robo_global2, robo_global4, posdia4,3);
    }
    return NULL;
}
//******************begin here*******************
//**********************end here************************
void* ThreadFunction4(void* data)
{
    while(1)
    {
        RoboControl *robo =(RoboControl*)data;
        robo->GotoXY(double((random()%200-100))/100.0, double((random()%200-100))/100.0, speed2, true);
        usleep(1000000);
        //while (robo2->GetPos().DistanceTo(posdia3) > 0.10) usleep(50000);
        //robo2->GotoXY(posdia4.GetX(), posdia4.GetY(), speed, true);
        //while (robo2->GetPos().DistanceTo(posdia4) > 0.10) usleep(50000);
    }

    return NULL;

}

int main(void) {
	//--------------------------------- Init --------------------------------------------------

	/** Use client number according to your account number!
	 *
	 *	This is necessary in order to assure that there are unique
	 *	connections to the RTDB.
	 *
	 */
        const int client_nr = 18;

        /** Type in the rfcomm number of the robo1t you want to connect to.
         *  The numbers of the robo1ts you are connected to can be found on the
	 *  screen when you connected to them.
	 *
         *  The red robo1ts' number will be in the range of 3 to 5
         *  The blue robo1ts' number will be in the range from 0 to 2
	 *
         *  The robo1ts are always connected to the lowest free rfcomm device.
         *  Therefore if you have two blue robo1ts connected the will be
	 *  connected to rfcomm number 0 and number 1...
	 *
	 */
        int rfcomm_nr1 = 0;
        int rfcomm_nr2 = 1;
        int rfcomm_nr3 = 2;
        int rfcomm_nr4 = 3;
/*        int rfcomm_nr5 = 4;
        int rfcomm_nr6 = 5;
*/
	try {

		/** Establish connection to the RTDB.
		 *
		 *  The connection to the RTDB is necessary in order to get access
                 *  to the control and the status of the robo1ts which are both stored
		 *  in the RTDB.
		 *
		 *  In the RTDB there are also informations about the ball and the
                 *  other robo1t positions.
		 *
		 */
		cout << endl << "Connecting to RTDB..." << endl;
		/** Create the client name with the unique client number*/
		string client_name = "pololu_client_";
		client_name.push_back((char) (client_nr + '0'));
		RTDBConn DBC(client_name.data(), 0.1, "");

                /** Create a new robo1Control object.
		 *
                 *  This is the basis for any communication with the robo1t.
		 *
		 *  We need to hand over the RTDB connection (DBC) and the rfcomm
                 *  number of the robo1t we want to control.
		 */
                static RoboControl robo1(DBC, rfcomm_nr1);
                static RoboControl robo2(DBC, rfcomm_nr2);
                RoboControl robo3(DBC, rfcomm_nr3);
                RoboControl robo4(DBC, rfcomm_nr4);
/*                RoboControl robo5(DBC, rfcomm_nr5);
                RoboControl robo6(DBC, rfcomm_nr6);
*/
                robo_global1 = &robo1;
                robo_global2 = &robo2;
                robo_global3 = &robo3;
                robo_global4 = &robo4;

                /** Now let's print out some information about the robo1t... */
                //uint8_t mac[6];
                //robo1.GetMac(mac);
                cout << "robo1 @ rfcomm" << rfcomm_nr1 << endl; /*<<" with Mac: ";


		for (int j = 0; j < 5; j++)
			cout << hex << (int) mac[j] << ":";
		cout << hex << (int) mac[5] << endl;
*/
                cout << "\t Battery Voltage: " << dec << (int) robo1.GetAccuVoltage()
                                << "mV" << endl;
                cout << "\t initial position: " << robo1.GetPos() << endl;
                cout << "\t initial rotation: " << robo1.GetPhi() << endl;

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

		/** Define four positions which form a rectangle...
		 *
		 */
		Position pos1(-0.6, -0.6);
		Position pos2(0.6, -0.6);
		Position pos3(0.6, 0.6);
		Position pos4(-0.6, 0.6);
                //Position roboPos;

                //test for random variable
                //double b=rand()/(RAND_MAX+1.0);
                //cout<<"a random number is "<<rand()/(RAND_MAX+1.0)<<" and "<< rand()/(RAND_MAX+1.0)<<endl<<endl;

/*
                pthread_t thread;
                pthread_create(&thread, NULL, ThreadFunction, &robo1);
                usleep(10e6);
                keepGoing = false;
                pthread_join(thread, NULL);
*/
                pthread_t thread1;
                pthread_create(&thread1, NULL, ThreadFunction1, NULL);

                pthread_t thread2;
                pthread_create(&thread2, NULL, ThreadFunction2, NULL);

                pthread_t thread3;
                pthread_create(&thread3, NULL, ThreadFunction3, NULL);

                pthread_t thread4;
                pthread_create(&thread4, NULL, ThreadFunction4, &robo4);
                /*
                while(1)
                {
                    avoidance(robo_global1, robo_global2, robo_global3, posdia1);
                    avoidance(robo_global1, robo_global2, robo_global3, posdia2);
                }
                */

                //coordinate for collision avoidance
                //double xavoid, yavoid;

                while (1) {
			/** Sequentially move to the four different positions.
			 *  The while is excited if the position is reached.
			 */
/*
			cout << "Moving to " << pos1 << endl << endl;
                        //robo1.GotoXY(pos1.GetX(), pos1.GetY(), speed, false);
                        robo1.GotoPos(same(pos1));
                        //robo1.GotoPos(collision(pos1, robo1, robo2));
                        while (robo1.GetPos().DistanceTo(pos1) > 0.10)
                        {
                            if(robo1.GetPos().DistanceTo(robo2.GetPos())<0.15)
                            //check if there is a potential collision and change temporary direction of the robot.
                            {
                                //random escape route; functional but not usefull
                                //robo1.GotoXY( (rand()%200-200)/100, (rand()%200-200)/100, speed, false);

                                //perpendicular escape route
                                xavoid=(robo2.GetY()-robo1.GetY());
                                yavoid=-(robo2.GetX()-robo1.GetX());
                                //find the perpendicular direction to the line between robot and the obstacle-robot
                                if((xavoid*(pos1.GetX()-robo1.GetX())+yavoid*(pos1.GetY()-robo1.GetY()))<0)
                                //because there are 2 perpendicular directions, choose the one that is close to final destination
                                {
                                        xavoid=-xavoid;
                                        yavoid=-yavoid;
                                }
                                cout<<"The escape direction of robo1 is x="<<xavoid<<", y="<<yavoid<<endl<<endl;
                                robo1.GotoXY( xavoid+robo1.GetX(), yavoid+robo1.GetY(), speed, false);
                            }
                            usleep(10000); //give the robot some time to run away from the obstacle.
                            robo1.GotoPos(same(pos1));
                            //after collision is avoided, turn to the final destination again.
                        }
                        //Camera sampling rate is 30fps -> 33ms
                        //which means that field info does not change within this time
                        cout<<"The distance between robo1 and robo2 is "<<robo1.GetPos().DistanceTo(robo2.GetPos())<<endl<<endl;


			cout << "Moving to " << pos2 << endl << endl;
                        robo1.GotoXY(pos2.GetX(), pos2.GetY(), speed, true);
                        while (robo1.GetPos().DistanceTo(pos2) > 0.10) usleep(50000);


			cout << "Moving to " << pos3 << endl << endl;
                        robo1.GotoXY(pos3.GetX(), pos3.GetY(), speed, false);
                        while (robo1.GetPos().DistanceTo(pos3) > 0.10)
                        {
                            if(robo1.GetPos().DistanceTo(robo2.GetPos())<0.15)
                            {
                                //random escape route; functional but not usefull
                                //robo1.GotoXY( (rand()%200-200)/100, (rand()%200-200)/100, speed, false);

                                //perpendicular escape route
                                xavoid=(robo2.GetY()-robo1.GetY());
                                yavoid=-(robo2.GetX()-robo1.GetX());
                                if((xavoid*(pos3.GetX()-robo1.GetX())+yavoid*(pos3.GetY()-robo1.GetY()))<0)
                                {
                                        xavoid=-xavoid;
                                        yavoid=-yavoid;
                                }
                                cout<<"The escape direction of robo1 is x="<<xavoid<<", y="<<yavoid<<endl<<endl;
                                robo1.GotoXY( xavoid+robo1.GetX(), yavoid+robo1.GetY(), speed, false);
                            }
                            usleep(10000); //sleep function in microseconds
                            robo1.GotoPos(same(pos3));
                        }

			cout << "Moving to " << pos4 << endl << endl;
                        robo1.GotoXY(pos4.GetX(), pos4.GetY(), speed, true);
                        //roboPos=robo1.GetPos();
                        while (robo1.GetPos().DistanceTo(pos4) > 0.10) usleep(50000);

                        //cout << "Ball position is"<<ball.GetPos()<<endl<<endl;
                        */
		}

	} catch (DBError err) {
		cout << "Client died on Error: " << err.what() << endl;
	}
        cout << "End" << endl;
	return 0;
}
Position same(Position target)
{
    return target;
}
