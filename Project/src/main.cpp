//============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   :
// Description :
//============================================================================


#include <time.h>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <sys/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "refereedisplay.h"
#include "interpreter.h"
#include "newrobocontrol.h"
#include "mapdisplay.h"
#include "goalkeeper.h"
#include "playermain.h"
#include "playertwo.h"
#include "opponentrobot.h"
#include "pathfinder.h"
#include "log.h"

#ifdef STACK_LOG
static int StackFd;
void exitHandler(int signal, siginfo_t* info, void* sigcontext)
{
    if (signal != SIGINT && signal != SIGTERM)
    {
        void* array[50];
        size_t size = backtrace(array, 50);

        // Unexpected termination - write the stack trace to a file
        backtrace_symbols_fd(array, size, StackFd);

        _exit(1);
    }

    // Normal termination
    _exit(0);
}
#endif

using namespace std;

typedef struct
{
  RefereeDisplay* refereeDisplay;
  Interpreter* info;
  TeamRobot* robo;
} MainLoopDataStruct;

static void* MainLoop(void* data);

const eTeam team = BLUE_TEAM;

#ifdef SIMULATION
const unsigned int refreshWait = 500;
#else
const unsigned int refreshWait = 20;
#endif

int main(void)
{

#ifdef STACK_LOG
    struct sigaction sa;
    StackFd = open ("stack_trace.log", O_WRONLY|O_CREAT, S_IRWXU);
    sa.sa_sigaction = exitHandler;
    sigemptyset (&sa.sa_mask);
    sa.sa_flags = SA_RESTART|SA_SIGINFO;

    sigaction (SIGINT, &sa, NULL);
    sigaction (SIGTERM, &sa, NULL);
    sigaction (SIGILL, &sa, NULL);
    sigaction (SIGSEGV, &sa, NULL);
    sigaction (SIGABRT, &sa, NULL);
#endif

    //--------------------------------- Init ---------------------------------
    srand(time(NULL));

    const int client_nr = 12;

    int rfcomm_nr_blue[] = {0, 1, 2};
    int rfcomm_nr_red[] = {3, 4, 5};

    int* rfcomm_nr = team == BLUE_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;
    int* rfcomm_nr_2 = team == RED_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;

    CoordinatesCalibrer coordCalibrer;
    #ifdef SIMULATION
    coordCalibrer.SetManualCoordCalibration(Position(0, -0.867), Position(1.367, 0), Position(0, 0.867), Position(-1.367, 0));              //Calibration settings for the simulation
    #else
    coordCalibrer.SetManualCoordCalibration(Position(-0.03,-0.826), Position(1.395,0.08), Position(-0.027,0.908), Position(-1.44,0.036));   //Calibration settings for the real field
    #endif

    BallMonitor ballMonitor(&coordCalibrer);
    RefereeDisplay refereeDisplay(&ballMonitor, &coordCalibrer);

    try
    {
        cout << endl;
        Log("Connecting to RTDB...", INFO);

        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RawBall ball(DBC);

        Goalkeeper gk = Goalkeeper(DBC, rfcomm_nr[0], &coordCalibrer, &ball, &ballMonitor);
        PlayerMain p1 = PlayerMain(DBC, rfcomm_nr[1], &coordCalibrer, &ball, &ballMonitor);
        PlayerTwo p2 = PlayerTwo(DBC, rfcomm_nr[2], &coordCalibrer, &ball, &ballMonitor);
        OpponentRobot robo4 = OpponentRobot(DBC, rfcomm_nr_2[0]);
        OpponentRobot robo5 = OpponentRobot(DBC, rfcomm_nr_2[1]);
        OpponentRobot robo6 = OpponentRobot(DBC, rfcomm_nr_2[2]);


        Referee ref(DBC);
        ref.Init();
        Log("Side: " + ToString(ref.GetSide()), INFO);

        ballMonitor.StartMonitoring(&ball);

        Interpreter info(team, &ref, &gk, &p1, &p2, &robo4, &robo5, &robo6, &ball, &coordCalibrer);

        NewRoboControl* robots[] = {&gk, &p1, &p2, &robo4, &robo5, &robo6};
        refereeDisplay.StartDisplay(robots, &info, &(p1.getMap()));
        p1.GiveDisplay(&refereeDisplay);

        //-------------------------------------- Ende Init ---------------------------------
        MainLoopDataStruct s1 = {&refereeDisplay, &info, &gk};
        MainLoopDataStruct s2 = {&refereeDisplay, &info, &p1};
        MainLoopDataStruct s3 = {&refereeDisplay, &info, &p2};

        pthread_t gkThread, p1Thread, p2Thread;
        pthread_create(&gkThread, NULL, MainLoop, &s1);
        pthread_create(&p1Thread, NULL, MainLoop, &s2);
        pthread_create(&p2Thread, NULL, MainLoop, &s3);

        while (refereeDisplay.IsActive())
        {
            Uint32 t0 = SDL_GetTicks();

            info.updateSituation();

            if (SDL_GetTicks() - t0 <= refreshWait)
                usleep((refreshWait - (SDL_GetTicks() - t0)) * 1000);
        }

        pthread_join(gkThread, NULL);
        pthread_join(p1Thread, NULL);
        pthread_join(p2Thread, NULL);
    }

    catch (DBError err)
    {
        Log(string("Client died on Error: ") + err.what(), ERROR);
    }

    refereeDisplay.StopDisplay();
    ballMonitor.StopMonitoring();

    Log("End", INFO);
    pthread_exit(NULL);
    return 0;
}


static void* MainLoop(void* data)
{
    MainLoopDataStruct* s = (MainLoopDataStruct*)data;
    int id = -1;

    while (s->refereeDisplay->IsActive())
    {
        id = s->info->waitForUpdate(id);

        s->robo->setNextCmd(s->info->getMode());
        s->robo->setCmdParam(*(s->info));
        s->robo->performCmd(s->info->getMode());
    }

    return NULL;
}
