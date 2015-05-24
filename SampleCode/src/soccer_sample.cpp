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
#include "interpreter.h"



using namespace std;



int main(void)
{
    //--------------------------------- Init -------------------- Can't connect RF------------------------------

    const int client_nr = 12;


    int team = 0;
    int rfcomm_nr[] = {0,1,2};
    pthread_t threads[3];
    //struct thread_data td[3];
    int gk_th,p1_th,p2_th;

    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RoboControl robo1 = RoboControl(DBC, rfcomm_nr[0]);
        RoboControl robo2 = RoboControl(DBC, rfcomm_nr[1]);
        RoboControl robo3 = RoboControl(DBC, rfcomm_nr[2]);


        RawBall ball(DBC);
        Referee ref(DBC);
        ref.Init();
        cout<< ref.GetSide()<<endl;

        Goalkeeper gk(&robo1,&ball);
        PlayerMain p1(&robo2,&ball);
        PlayerTwo p2(&robo3,&ball);

        interpreter info(team,&ref,&gk,&p1,&p2);


        //-------------------------------------- Ende Init ---------------------------------



        while (1)
        {
            info.updateSituation();
            if(info.turn == interpreter::OUR_TURN)
            {
                cout<<"OUR TURN"<<endl;
                usleep(1000000);
            }
            else
            {    cout << "NOT OUR TURN"<<endl;
                 usleep(1000000);

            }
            gk.setNextCmd(&info);
            gk.setCmdParam();

            p1.setNextCmd(&info);
            p1.setCmdParam();

            p2.setNextCmd(&info);
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

    cout << "End" << endl;
    pthread_exit(NULL);
    return 0;
}

