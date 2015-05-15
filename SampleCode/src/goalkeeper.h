
#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_


#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"



class Goalkeeper {
	enum ActionGk{GO_TO_DEF_POS,PREVENT_GOAL};


private:

        RawBall* ball;

	ActionGk nextCmd;
        Position preventGoalParam;

public:
	Position defaultPos;
        RoboControl* robot;

        void setNextCmd(void*);
	void setCmdParam();
	void performCmd();
        Goalkeeper(RoboControl*,RawBall*);
	~Goalkeeper();
};

#endif /* GOALKEEPER_H_ */
