#ifndef STANDARDFSM_H
#define STANDARDFSM_H

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "referee.h"
#include "newrobocontrol.h"

typedef struct
{
    NewRoboControl *robo;
    RawBall *ball;
    Referee *ref;
} RoboBall;

typedef void (*PlayFunc)(NewRoboControl**, RawBall*, Referee*);

void StandardFSM(NewRoboControl *robots[], RawBall *ball, Referee *ref);

#endif // STANDARDFSM_H
