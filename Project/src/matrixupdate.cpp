#include <iostream>
#include "kogmo_rtdb.hxx"
#include "robo_control.h"
#include "coordinates.h"
#include "interpreter.h"
#include "referee.h"
using namespace std;
// function that updates the matrix
 int** matrixupdate(int h,int w, int** matrix, RoboControl *robots[6], RawBall *ball, CoordinatesCalibrer *coordCalibrer, eSide our_side);
 int main()
 {   const int client_nr = 15;
     int rfcomm_nr_blue[] = {0, 1, 2};
     int rfcomm_nr_red[] = {3, 4, 5};

     int *rfcomm_nr = team == BLUE_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;
     //int *rfcomm_nr_2 = team == RED_TEAM ? rfcomm_nr_blue : rfcomm_nr_red;

     try
     {
         cout << endl << "Connecting to RTDB..." << endl;
         string client_name = "pololu_client_";
         client_name.push_back((char)(client_nr + '0'));
         RTDBConn DBC(client_name.data(), 0.1, "");
    int initialMatrix[5][5];

    RoboControl robo1 = RoboControl(DBC, rfcomm_nr[0]);
    RoboControl robo2 = RoboControl(DBC, rfcomm_nr[1]);
    RoboControl robo3 = RoboControl(DBC, rfcomm_nr[2]);
    RoboControl robo4 = RoboControl(DBC, rfcomm_nr_2[0]);
    RoboControl robo5 = RoboControl(DBC, rfcomm_nr_2[1]);
    RoboControl robo6 = RoboControl(DBC, rfcomm_nr_2[2]);
    RoboControl *robots[6] = {&robo1, &robo2, &robo3, &robo4, &robo5, &robo6};
    RawBall ball(DBC);
    Referee ref(DBC);
    ref.Init();
    CoordinatesCalibrer coordCalibrer= CoordinatesCalibrer();
    eSide ourSide= LEFT_SIDE;
    int[][]updatedMatrix= matrixupdate(5,5,initialMatrix,*robots[6],*ball,coordCalibrer,ourSide);
     }
     catch (DBError err)
     {
         cout << "Client died on Error: " << err.what() << endl;
     }

     cout << "End" << endl;
     return 0;
 }

int** matrixupdate(int h,int w, int** matrix, RoboControl *robots[6], RawBall *ball, CoordinatesCalibrer *coordCalibrer, eSide our_side)
{
 int[][]newMatrix= matrix;
 m_coordCalibrer = coordCalibrer;
 double l=2/(h-1);
 double k=2/(w-1);
 //Normalize coordinates of our robot
 Position pos = m_coordCalibrer->NormalizePosition(robots[0].GetPos());//robots[0] is our robot
 //indices of the robot's position in the matrix
 int i=static_cast<int>(floor(pos.GetY()/l)-floor(-1/l));
 int j=static_cast<int>(floor(pos.GetX()/k)-floor(-1/k));
 newMatrix[i][j]=2; // affect 2 to the position of our robot in the matrix
//Normalize coordinates of the ball
 Position pos = m_coordCalibrer->NormalizePosition(ball.GetPos());
 //indices of the robot's position in the matrix
 int i=static_cast<int>(floor(pos.GetY()/l)-floor(-1/l));
 int j=static_cast<int>(floor(pos.GetX()/k)-floor(-1/k));
 newMatrix[i][j]=3; // affect 3 to the position of the ball in the matrix
 //generate the obstacles around the ball depending on the side in which our team plays
 newMatrix[i-1][j-1]=1;
 newMatrix[i-1][j]=1;
 newMatrix[i-1][j+1]=1;
 newMatrix[i+1][j-1]=1;
 newMatrix[i+1][j]=1;
 newMatrix[i+1][j+1]=1;
 if (our_side == LEFT_SIDE)
  newMatrix[i][j+1]=1;
 else
  newMatrix[i][j-1]=1;
 for (int i=1 ; i < 6 ; i++)
 {
    //Normalize coordinates of the obstacles
    Position pos = m_coordCalibrer->NormalizePosition(robots[i].GetPos());//robots[i] is an obstacle
    //indices of the robot's position in the matrix
    int i=static_cast<int>(floor(pos.GetY()/l)-floor(-1/l));
    int j=static_cast<int>(floor(pos.GetX()/k)-floor(-1/k));
    newMatrix[i][j]=1; // affect 1 to the position of the obstacle and its surroundings in the matrix
    newMatrix[i-1][j]=1;
    newMatrix[i+1][j]=1;
    newMatrix[i][j-1]=1;
    newMatrix[i][j+1]=1;
}
 return newMatrix;
}
