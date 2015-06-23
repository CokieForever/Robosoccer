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
#include "kogmo_rtdb.hxx"
#include "referee.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "refereedisplay.h"
#include "interpreter.h"
#include "newrobocontrol.h"
#include "matrixdisplay.h"
#include "goalkeeper.h"
#include "playermain.h"
#include "playertwo.h"
#include "opponentrobot.h"

using namespace std;


const eTeam team = BLUE_TEAM;

int main(void)
{
    //--------------------------------- Init ---------------------------------
    const int client_nr = 11;

    int rfcomm_nr_blue[] = {0, 1, 2};
    int rfcomm_nr_red[] = {3, 4, 5};

    int *rfcomm_nr = team == BLUE_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;
    int *rfcomm_nr_2 = team == RED_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;

    pthread_t threads[3];
    //struct thread_data td[3];
    int gk_th,p1_th,p2_th;

    CoordinatesCalibrer coordCalibrer;
    coordCalibrer.SetManualCoordCalibration(Position(-0.03,-0.826), Position(1.395,0.08), Position(-0.027,0.908), Position(-1.44,0.036));     //Calibration settings for the real field
    //coordCalibrer.SetManualCoordCalibration(Position(0,-0.867), Position(1.367,0), Position(0,0.867), Position(-1.367,0));                  //Calibration settings for the simulation

    BallMonitor ballMonitor(&coordCalibrer);
    RefereeDisplay refereeDisplay(team, &ballMonitor, &coordCalibrer);

    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RawBall ball(DBC);

        Goalkeeper gk = Goalkeeper(DBC, rfcomm_nr[0], &coordCalibrer, &ball);
        PlayerMain p1 = PlayerMain(DBC, rfcomm_nr[1], &coordCalibrer, &ball);
        PlayerTwo p2 = PlayerTwo(DBC, rfcomm_nr[2], &coordCalibrer, &ball);
        OpponentRobot robo4 = OpponentRobot(DBC, rfcomm_nr_2[0]);
        OpponentRobot robo5 = OpponentRobot(DBC, rfcomm_nr_2[1]);
        OpponentRobot robo6 = OpponentRobot(DBC, rfcomm_nr_2[2]);

        NewRoboControl *robots[] = {&gk, &p1, &p2, &robo4, &robo5, &robo6};

        Referee ref(DBC);
        ref.Init();
        cout << ref.GetSide() <<endl;

        MatrixDisplay::Matrix matrix = MatrixDisplay::ConvertToMatrix(&(p1.getMap()[0][0]), Interpreter::MAP_WIDTH, Interpreter::MAP_HEIGHT);

        ballMonitor.StartMonitoring(&ball);
        refereeDisplay.StartDisplay(robots, &ball, &matrix);

        Interpreter info(team,&ref,&gk,&p1,&p2,&robo4,&robo5,&robo6,&ball,&coordCalibrer);


        //-------------------------------------- Ende Init ---------------------------------

        //plan path for p1
        /*
        Point A,B;
        A.x = 2;
        A.y = 5;

        B.x = 17;
        B.y = 10;

        showMap(p1.map,pathFind(p1.map,A,B),A);
        */
        while (refereeDisplay.IsActive())
        {
            info.updateSituation();

            gk.setCmdParam();

            p1.setNextCmd(&info);
            p1.setCmdParam();

            p2.setCmdParam();

            gk_th = pthread_create(&threads[0],NULL,&Goalkeeper::performCmd_helper,(void*)&gk);
            p1_th = pthread_create(&threads[1],NULL,&PlayerMain::performCmd_helper,(void*)&p1);
            p2_th = pthread_create(&threads[2],NULL,&PlayerTwo::performCmd_helper,(void*)&p2);

            usleep(33000);

        }

    }
    catch (DBError err)
    {
        cout << "Client died on Error: " << err.what() << endl;
    }

    refereeDisplay.StopDisplay();
    ballMonitor.StopMonitoring();

    cout << "End" << endl;
    pthread_exit(NULL);
    return 0;
}
