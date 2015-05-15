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
#include "goalkeeper.h"
#include "interpreter.h"


using namespace std;



int main(void)
{
    //--------------------------------- Init -------------------- Can't connect RF------------------------------

    const int client_nr = 12;


    int team = 0;
    int rfcomm_nr[] = {0,1,2};

    try
    {
        cout << endl << "Connecting to RTDB..." << endl;
        string client_name = "pololu_client_";
        client_name.push_back((char)(client_nr + '0'));
        RTDBConn DBC(client_name.data(), 0.1, "");

        RoboControl robo1 = RoboControl(DBC, rfcomm_nr[0]);



        RawBall ball(DBC);
        Referee ref(DBC);
        ref.Init();
        cout<< ref.GetSide()<<endl;

        Goalkeeper gk(&robo1,&ball);
        interpreter info(team,&ref,&gk);
        std::cout << "hello" <<endl;

        //-------------------------------------- Ende Init ---------------------------------


        while (1)
        {
            cout<<info.turn<<endl;
            usleep(1000000);
        }

    }
    catch (DBError err)
    {
        cout << "Client died on Error: " << err.what() << endl;
    }

    cout << "End" << endl;
    return 0;
}

