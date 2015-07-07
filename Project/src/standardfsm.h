#ifndef STANDARDFSM_H
#define STANDARDFSM_H

#include <time.h>
#include <iostream>
#include <pthread.h>
#include "referee.h"
#include "newrobocontrol.h"

/**
 * @brief
 *
 */
typedef struct
{
    NewRoboControl *robo;   /**< TODO */
    RawBall *ball;          /**< TODO */
    Referee *ref;           /**< TODO */
} RoboBall;

/**
 * @brief
 *
 */
typedef void (*PlayFunc)(NewRoboControl**, RawBall*, Referee*);

void StandardFSM(NewRoboControl *robots[], RawBall *ball, Referee *ref);

#endif // STANDARDFSM_H
