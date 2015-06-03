#ifndef STANDARDFSM_H
#define STANDARDFSM_H

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "robo_control.h"
#include "referee.h"

typedef struct
{
    RoboControl *robo;
    RawBall *ball;
    Referee *ref;
} RoboBall;

typedef void (*PlayFunc)(RoboControl**, RawBall*, Referee*);

void StandardFSM(RoboControl *robots[], RawBall *ball, Referee *ref);

#endif // STANDARDFSM_H
