//============================================================================
// Name        : soccer_client.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Client for RTDB which controls team 1, Ansi-style
//============================================================================

#include <vector>
#include <string>
#include <time.h>
#include <iostream>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include <tulip/Graph.h>
using namespace std;
using namespace tlp;

int main(void) {
	//--------------------------------- Init --------------------------------------------------

	/** Use client number according to your account number!
	 *
	 *	This is necessary in order to assure that there are unique
	 *	connections to the RTDB.
	 *
	 */
        const int client_nr = 15;

	/** Type in the rfcomm number of the robot you want to connect to.
	 *  The numbers of the robots you are connected to can be found on the
	 *  screen when you connected to them.
	 *
	 *  The red robots' number will be in the range of 3 to 5
	 *  The blue robots' number will be in the range from 0 to 2
	 *
	 *  The robots are always connected to the lowest free rfcomm device.
	 *  Therefore if you have two blue robots connected the will be
	 *  connected to rfcomm number 0 and number 1...
	 *
	 */
        int rfcomm_nr = 0;

	try {

		/** Establish connection to the RTDB.
		 *
		 *  The connection to the RTDB is necessary in order to get access
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
		RoboControl robo(DBC, rfcomm_nr);

		/** Now let's print out some information about the robot... */

                cout << "Robo @ rfcomm" << rfcomm_nr << endl;
                cout << "\t initial position: " << robo.GetPos() << endl;
		cout << "\t initial rotation: " << robo.GetPhi() << endl;

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
		 *  Therefore if the ball does not move the rotation is not defined.
		 */
		cout << "\t initial direction: " << ball.GetPhi() << endl;
		cout << "\t initial velocity: " << ball.GetVelocity() << endl;

		//-------------------------------------- Ende Init ---------------------------------
                //create a matrix representing the playground
                int i,j;
                int matrix[36][58];
                for (i = 0; i <= 35; i++)
                    for (j=0; j<=57; j++)
                    {matrix[i][j]=0;}
                //create an empty graph
                 Graph *graph = tlp::newGraph();
                 //create a list of nodes
                 node all_nodes[36][58];

                 //add  nodes
                 for (i=0; i<=35;i++)
                      for (j=0; j<=57; j++)
                      {
                         all_nodes[i][j] = graph->addNode();

                      }

                 //add edges
                 for (i = 1; i <= 34; i++)
                     for (j=1; j<=56; j++)
                 edge e1 = graph->addEdge(all_nodes[i][j],all_nodes[i+1][j]);
                 edge e2 = graph->addEdge(all_nodes[i][j],all_nodes[i+][j+1]);
                 edge e3 = graph->addEdge(all_nodes[i][j],all_nodes[i-1][j]);
                 edge e4 = graph->addEdge(all_nodes[i][j],all_nodes[i][j-1]);

                 //print the result on the standard output
                   cout << graph << flush ;

                  // tlp::saveGraph(graph,"tuto1.tlp");

                cout <<  "initial ball pos " << ball.GetPos() << endl ;
                while (1)
                {
                    ball_x=ball.GetX();
                    ball_y=ball.GetY();
                    int i=static_cast<int>(floor(ball_y/5)+17);
                    int j=static_cast<int>(floor(ball_x/5)+29);
                    matrix[i][j]=matrix[i][j]+10;

        }

}
        catch (DBError err)
        {
		cout << "Client died on Error: " << err.what() << endl;
	}

        cout << "End" << endl;
	return 0;

}


