
#ifndef GOALKEEPER_H_
#define GOALKEEPER_H_



#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "cruiseToBias2.h"
#include "ballmonitor.h"



class Goalkeeper {
	enum ActionGk{GO_TO_DEF_POS,PREVENT_GOAL};


private:

	RawBall* ball;

	ActionGk nextCmd;
        Position preventGoalParam;
	BallMonitor ballMonitor;

public:
        Goalkeeper(RoboControl *x, RawBall *b, CoordinatesCalibrer *coordCalibrer, RobotMonitor *robotMonitor);
        
	Position defaultPos;
        RoboControl* robot;

        void setNextCmd(void*);
	void setCmdParam();
        void *performCmd(void);
        static void *performCmd_helper(void *);

	~Goalkeeper();
};

#endif /* GOALKEEPER_H_ */
