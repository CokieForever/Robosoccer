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

typedef struct
{
    RoboControl robo;
    RawBall ball;
} RoboBall;


using namespace std;

void* RedMove(void *data);

int main(void) {
	//--------------------------------- Init --------------------------------------------------

	/** Use client number according to your account number!
	 *
	 *	This is necessary in order to assure that there are unique
	 *	connections to the RTDB.
	 *
	 */
        const int client_nr = 11;

	/** Type in the rfcomm number of the robot you want to connect to.
	 *  The numbers of the robots you are connected to can be found on the
	 *  screen when you connected to them.
	 *
	 *  The red robots' number will be in the range of 3 to 5
	 *  The blue robots' number will be in the range from 0 to 2
	 *
	 *  The robots are always connected to the lowest free rfcomm device.
	 *  Therefore if you have two blue robots connected the will be
         *  connectedvoid RedMove(RawBall ball, RoboControl redRobo) to rfcomm number 0 and number 1...
	 *
	 */
        int rfcomm_nr = 0;
        int rfcomm_nr_2 = 1;


/*
        //Activate the project option "Run in terminal" in Ctrl+5 (Ctrl+2 to come back here)
        //to use "cin" function.

        char cInput;

        cout << "Specify rfcomm number of the robot you want to move. Blue 0-2, Red 3-5: ";
	while (1) {                
                cin >> cInput;
                if (!((cInput >= '0') && (cInput <= '7'))) {
			cout << "Please specify valid rfcomm number between 0 and 7"
					<< endl;
		} else {
                        rfcomm_nr = cInput-48; //simple "char to int" function
			break;
		}
        }
*/
	try {

		/** Establish connection to the RTDB.
		 *
                 *  The connection to the RTDB is #include <thread>necessary in order to get access
		 *  to the control and the status of the robots which are both stored
		 *  in the RTDB.
		 *
		 *  In the RTDB there are also informations about the ball and the
		 *  other robot positions.
		 *
		 */
		cout << endl << "Connecting to RTDB..." << endl;
		/** Create the client name with the unique client number*/
		string client_name = "pololu_client_";
		client_name.push_back((char) (client_nr + '0'));
		RTDBConn DBC(client_name.data(), 0.1, "");

		/** Create a new RoboControl object.
		 *
		 *  This is the basis for any communication with the robot.
		 *
		 *  We need to hand over the RTDB connection (DBC) and the rfcomm
		 *  number of the robot we want to control.
		 */
                RoboControl blueRobo(DBC, rfcomm_nr);
                RoboControl redRobo(DBC, rfcomm_nr_2);

		/** Now let's print out some information about the robot... */
                //uint8_t mac[6];
                //robo.GetMac(mac);
                cout << "Robo @ rfcomm" << rfcomm_nr << endl; /*<<" with Mac: ";


		for (int j = 0; j < 5; j++)
			cout << hex << (int) mac[j] << ":";
		cout << hex << (int) mac[5] << endl;
*/
                cout << "\t Battery Voltage: " << dec << (int) blueRobo.GetAccuVoltage()
                                << "mV" << endl;
                cout << "\t initial position: " << blueRobo.GetPos() << endl;
                cout << "\t initial rotation: " << blueRobo.GetPhi() << endl;

		/** Create a ball object
		 *
		 *  This ball abject gives you access to all information about the ball
		 *  which is extracted from the cam.
		 *
		 */
		RawBall ball(DBC);
		/** lets print this information: */
		cout << "Ball informations:" << endl;
		cout << "\t initial position: " << ball.GetPos() << endl;
		/** Notice that the rotation here refers to the moving direction of the ball.
                 *  Therefore if the balltargetPos does not move the rotation is not defined.
		 */
		cout << "\t initial direction: " << ball.GetPhi() << endl;
		cout << "\t initial velocity: " << ball.GetVelocity() << endl;

		//-------------------------------------- Ende Init ---------------------------------

                pthread_t thread1;
                RoboBall roboBall = {redRobo, ball};
                pthread_create( &thread1, NULL, RedMove, &roboBall);

		while (1) {

                        Position bluePos = blueRobo.GetPos();
                        Position ballPos = ball.GetPos();

                        double deltaX = fabs(bluePos.GetX() - ballPos.GetX());
                        double deltaY = fabs(bluePos.GetY() - ballPos.GetY());
                        double phi = atan2(deltaY, deltaX) * 180 / M_PI;
                        cout << "Phi = " << phi << endl;

                        if (phi >= 5)
                        {
                            double y = ballPos.GetY();

                            if (y > 0.15)
                                y = 0.15;
                            else if (y < -0.15)
                                y = -0.15;

                            deltaY = fabs(bluePos.GetY() - y);

                            cout << "Blue moving to y = " << y << endl;
                            blueRobo.GotoXY(-1.300, y, 160 * deltaY / 0.3, false);
                        }

                        usleep(30000);
		}

	} catch (DBError err) {
		cout << "Client died on Error: " << err.what() << endl;
	}
        cout << "End" << endl;
	return 0;
}

void* RedMove(void *data)
{
    RoboBall *roboBall = (RoboBall*)data;

    while (1)
    {
        Position ballPos = roboBall->ball.GetPos();
        roboBall->robo.GotoXY(ballPos.GetX(), ballPos.GetY(), 120, false);
        cout << "Red moving to " << ballPos << endl << endl;
        usleep(2000000);
    }

    return NULL;
}


