/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "interpreter.h"
#include <iostream>
#include "share.h"
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "ballmonitor.h"




Goalkeeper::Goalkeeper(RoboControl *x,RawBall *b, CoordCalibrer *coordCalibrer, RobotMonitor *robotMonitor, RawBall *ball) : ballMonitor(coordCalibrer, robotMonitor) {
	robot = x;
	ball  = b;
	
	ballMonitor.StartMonitoring(ball)
}

Goalkeeper::~Goalkeeper(){
}

void Goalkeeper::setNextCmd(void *s){

        interpreter* info = (interpreter*)s;
        switch(info->playmode)
	{
                case PENALTY:
                        if(info->turn != interpreter::OUR_TURN)
                            nextCmd = PREVENT_GOAL;
                        else
                            nextCmd = GO_TO_DEF_POS;
                        break;
		case PLAY_ON:
                        nextCmd = PREVENT_GOAL;
                        break;
		default:
                        nextCmd = GO_TO_DEF_POS;
                        break;
	}

}


void Goalkeeper::setCmdParam(){
    Position predballPos_old, predballPos_new, realballPos;
    static int counter = 0;
    double y;
    double deltaY;
    cout << "switch" << nextCmd << endl;
    switch(nextCmd)
        {
		case PREVENT_GOAL:  
//counter 10 predictballpos(..., 5) worked good
                    if(counter >= 9)
                   {
                        ballMonitor.PredictBallPosition(& predballPos_old, 5);
                        cout << "predictball" << predballPos_old << endl;

                        y = predballPos_old.GetY();
                        //define Goal borders
                        if (y > 0.22)
                             y = 0.22;
                         else if (y < -0.15)
                             y = -0.15;

                         deltaY = fabs(robot->GetY() - y);
                         cout << "Goal keeper moving to y = " << y << std::endl;
                         preventGoalParam.SetX(defaultPos.GetX());
                         preventGoalParam.SetY(y);
                   counter =0;
                    }
                     counter ++;
                    break;

                case GO_TO_DEF_POS:
                        std::cout << " = " << defaultPos <<std::endl;
                        break;

                default :
			break;
	}
}



void *Goalkeeper::performCmd(void){

	switch(nextCmd)
	{
                case Goalkeeper::PREVENT_GOAL:
                        std::cout << "Next command Prevent Goal Position: " << preventGoalParam.GetPos()<< std::endl;

                        CruisetoBias(defaultPos.GetX(),preventGoalParam.GetY(), 650, -10, 30, robot);

                        break;
                case Goalkeeper::GO_TO_DEF_POS:

                        robot->GotoXY(defaultPos.GetX(),defaultPos.GetY());
                      /*  while(robot->GetPos().DistanceTo(defaultPos)> 0.01)
                        {
                           usleep(10000000);
                        }*/

			break;
		default:
			break;
	}
        return 0;

}

void *Goalkeeper::performCmd_helper(void *context){

    return ((Goalkeeper*)context)->performCmd();

}





