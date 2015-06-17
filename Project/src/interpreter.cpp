/*
 * interpreter.cpp
 *
 *  Created on: Apr 26, 2015
 *      Author: dean
 */

#include "interpreter.h"
#include "referee.h"
#include <iostream>
#include "share.h"
#include "goalkeeper.h"
#include "playermain.h"
#include "playertwo.h"

//include the libs from sample code



interpreter::interpreter(int x,Referee *y,Goalkeeper *z,PlayerMain *p,PlayerTwo *t) {
	// TODO Auto-generated constructor stub
	team = x;
	ref  = y;
	gk = z;
        p1 = p;
        p2 = t;


	playmode = ref->GetPlayMode();


	if (x== 0)
		std::cout<<"We are team blue!"<<std::endl;

	else
		std::cout<<"We are team red! "<<std::endl;

	std::cout<<"Interpreter initialized"<<std::endl;
}

interpreter::~interpreter() {
	// TODO Auto-generated destructor stub
}


bool interpreter::verifyPos(){
	//check if all robots are on their default position and orientation
       if ((gk->robot->GetPos().DistanceTo(gk->defaultPos)< 0.06) && (p1->robot->GetPos().DistanceTo(p1->defaultPos)< 0.06)&& (p2->robot->GetPos().DistanceTo(p2->defaultPos)< 0.06))
            return 1;

        else
                    return 0;

}


void interpreter::setDefaultPos(){
	//sets default position struct depending on game mode of all robots to predefined values
        switch(ref->GetPlayMode())
	{

                case BEFORE_PENALTY:
                                if((turn == interpreter::OUR_TURN))
				{	
                                        gk->defaultPos.SetX(-0.3);
                                        gk->defaultPos.SetY(0.4);

                                        p1->defaultPos.SetX(0.0);
                                        p1->defaultPos.SetY(0.0);

                                        p2->defaultPos.SetX(-1.0);
                                        p2->defaultPos.SetY(-0.5);
				}
	

                                else if ((turn == interpreter::NOT_OUR_TURN))
				{	
                                    gk->defaultPos.SetX(1.2);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(-0.5);
                                    p1->defaultPos.SetY(-0.5);

                                    p2->defaultPos.SetX(-1.0);
                                    p2->defaultPos.SetY(-0.5);

				}
                                else
                                {
                                    gk->defaultPos.SetX(-0.3);
                                    gk->defaultPos.SetY(0.4);

                                    p1->defaultPos.SetX(0.0);
                                    p1->defaultPos.SetY(0.4);


                                    p2->defaultPos.SetX(-1.0);
                                    p2->defaultPos.SetY(-0.5);

                                }
                                break;

		case BEFORE_KICK_OFF:

                                if(our_side == LEFT_SIDE)
                                {
                                    gk->defaultPos.SetX(-1.1);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(-0.3);
                                    p1->defaultPos.SetY(-0.2);

                                    p2->defaultPos.SetX(-0.3);
                                    p2->defaultPos.SetY(0.2);



				}
	
                                else if(our_side == RIGHT_SIDE)
				{	
                                    gk->defaultPos.SetX(1.1);
                                    gk->defaultPos.SetY(0.0);

                                    p1->defaultPos.SetX(0.3);
                                    p1->defaultPos.SetY(0.2);

                                    p2->defaultPos.SetX(0.3);
                                    p2->defaultPos.SetY(-0.2);

				}
                                else
                                {
                                    gk->defaultPos.SetX(-1.0);
                                    gk->defaultPos.SetY(0.4);

                                    p1->defaultPos.SetX(0.2);
                                    p1->defaultPos.SetY(0.4);


                                    p2->defaultPos.SetX(0.2);
                                    p2->defaultPos.SetY(-0.4);
                                }

                                break;
		
		case PLAY_ON:
                        if(our_side == LEFT_SIDE)
                        {
                            gk->defaultPos.SetX(-1.1);
                            gk->defaultPos.SetY(0.0);

                            p1->defaultPos.SetX(-0.3);
                            p1->defaultPos.SetY(-0.2);

                            p2->defaultPos.SetX(-0.3);
                            p2->defaultPos.SetY(0.2);



                        }

                        else if(our_side == RIGHT_SIDE)
                        {
                            gk->defaultPos.SetX(1.1);
                            gk->defaultPos.SetY(0.0);

                            p1->defaultPos.SetX(0.3);
                            p1->defaultPos.SetY(0.2);

                            p2->defaultPos.SetX(0.3);
                            p2->defaultPos.SetY(-0.2);

                        }
			//distinguish between ATK and DEF mode 
			//tbd
			break;
                case PENALTY:
                        if (turn == NOT_OUR_TURN)
                                gk->defaultPos.SetX(-1.43);
                        break;
                case KICK_OFF:
                        break;

		default:
                        gk->defaultPos.SetX(1.1);
                        gk->defaultPos.SetY(0.0);

                        p1->defaultPos.SetX(0.3);
                        p1->defaultPos.SetY(0.2);

                        p2->defaultPos.SetX(0.3);
                         p2->defaultPos.SetY(-0.2);
                        //states such "as referee init, kick_off/penalty -> defpos doesnt change
			break;
	}
}



void interpreter::setPlayMode() {

	playmode = ref->GetPlayMode();

        if (playmode==BEFORE_KICK_OFF || playmode==BEFORE_PENALTY )
	{
                setSide();
		setTurn();

		if (verifyPos())               
                ref->SetReady(team);

	}

        /*else if (playmode==BEFORE_PENALTY)
	{
                setSide();
                setTurn();

	}




	 *
	 * 	other playmodes and their actions need to be defined
	 *
	 */

}



void interpreter::setSide(){

	eSide tmp_side = ref->GetBlueSide();
	// 0<-> blue 1<->red
	if(((team == 0) && (tmp_side==LEFT_SIDE)) || ((team == 1) && (tmp_side  == RIGHT_SIDE)))
		our_side = LEFT_SIDE;
	else
		our_side = RIGHT_SIDE;

}


void interpreter::setTurn(){
        if (our_side==ref->GetSide())
	{
		turn = OUR_TURN;
	}
	else
	{
		turn = NOT_OUR_TURN;
	}

}

void interpreter::updateSituation(){
	setPlayMode();
	setDefaultPos();

}
