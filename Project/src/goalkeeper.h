
#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_



#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"



class Goalkeeper {
        enum ActionGk{GO_TO_DEF_POS,PREVENT_GOAL,FOLLOWPATH};


private:

        RawBall* ball;

	ActionGk nextCmd;
        Position preventGoalParam;

public:
	Position defaultPos;
        RoboControl* robot;

        void setNextCmd(void*);
	void setCmdParam();
        void *performCmd(void);
        Goalkeeper(RoboControl*,RawBall*);
        static void *performCmd_helper(void *);

	~Goalkeeper();
};

#endif /* GOALKEEPER_H_ */
