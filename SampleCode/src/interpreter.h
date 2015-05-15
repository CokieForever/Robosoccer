
#ifndef INTERPRETER_H_
#define INTERPRETER_H_

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "goalkeeper.h"



class interpreter {



private:

        Referee *ref;
        Goalkeeper *gk;
        //PlayerMain *p1;
        //PlayerSecondary *p2;

	void setPlayMode();
	void setSide();
	void setTurn();
	bool verifyPos();
	void setDefaultPos();

public:
        enum kick_turn{OUR_TURN,NOT_OUR_TURN};
	int team;
	ePlayMode playmode;
	eSide our_side;
	kick_turn turn;


	void updateSituation();
	interpreter(int,Referee*,Goalkeeper*);
	~interpreter();
};

#endif /* INTERPRETER_H_ */
