//============================================================================
// Name        : soccer_client.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Client for RTDB which controls team 1, Ansi-style
//============================================================================


#include <time.h>
#include <iostream>
#include <pthread.h>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "referee.h"
#include "coordinates.h"
#include "ballmonitor.h"
#include "refereedisplay.h"
#include "robotmonitor.h"
#include "interpreter.h"
#include "cruiseToBias2.h"
#include "matrixdisplay.h"
#include "goalkeeper.h"
#include "playermain.h"
#include "playertwo.h"

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

    RobotMonitor robotMonitor(&coordCalibrer);
    BallMonitor ballMonitor(&coordCalibrer, &robotMonitor);
    RefereeDisplay refereeDisplay(team, &ballMonitor, &coordCalibrer);


    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RoboControl robo1 = RoboControl(DBC, rfcomm_nr[0]);
        RoboControl robo2 = RoboControl(DBC, rfcomm_nr[1]);
        RoboControl robo3 = RoboControl(DBC, rfcomm_nr[2]);
        RoboControl robo4 = RoboControl(DBC, rfcomm_nr_2[0]);
        RoboControl robo5 = RoboControl(DBC, rfcomm_nr_2[1]);
        RoboControl robo6 = RoboControl(DBC, rfcomm_nr_2[2]);

        RoboControl *robots[] = {&robo1, &robo2, &robo3, &robo4, &robo5, &robo6};
        robotMonitor.SetAllRobots(robots);

        RawBall ball(DBC);
        Referee ref(DBC);
        ref.Init();
        cout << ref.GetSide() <<endl;

        Goalkeeper gk(&robo1,&ball);
        PlayerMain p1(&robo2,&ball,&coordCalibrer);
        PlayerTwo p2(&robo3,&ball);

        MatrixDisplay::Matrix matrix = MatrixDisplay::ConvertToMatrix(&(p1.getMap()[0][0]), Interpreter::WIDTH, Interpreter::HEIGHT);

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
        while (1)
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
