#include "penaltyschooter.h"
#include "newrobocontrol.h"
#include <iostream>
#include "share.h"
#include "kogmo_rtdb.hxx"
#include "sdlutilities.h"
#include "coordinates.h"
#include"cruise2_2.h"


using namespace std;

//penaltyschooter::penaltyschooter(RoboControl robo, RawBall ball)
void penaltyschooter(RoboControl robo, RawBall ball)
{
  //   shootPenalty;
  // CoordinatesCalibrer coordCalibrer= CoordinatesCalibrer();
  // CoordinatesCalibrer m_coordCalibrer = coordCalibrer;
  /* Position pos1= Position(-1.41,-0.188);
   Position pos2= Position(-1.41,0.183);
   Position pos3= Position(1.41,-0.182);
   Position pos4= Position(1.395,0.193);*/
    if( ball.GetX()<0)
 { Position pos5 = Position(-1.41, 0.0);


  // Position pos1n=m_coordCalibrer->NormalizePosition(pos1);
  //Position pos2n=m_coordCalibrer->NormalizePosition(pos2);
  //Position pos3n=m_coordCalibrer->NormalizePosition(pos3);
  //Position pos4n=m_coordCalibrer->NormalizePosition(pos4);
  // Position pos5n=m_coordCalibrer->NormalizePosition(pos5);
  //Position pos6n=m_coordCalibrer->NormalizePosition(pos6);

  double d = pos5.DistanceTo(ball.GetPos());
  double cosAngle, sinAngle;
  double targetx, targety;
  Position target = Position(targetx, targety);
  ComputeLineAngle(pos5.GetX(), pos5.GetY(), ball.GetX(), ball.GetY(), &cosAngle, &sinAngle);
  ComputeVectorEnd(pos5.GetX(), pos5.GetY(), cosAngle, sinAngle, d + 0.10, &targetx, &targety);
  robo.GotoXY(targetx, targety);
  for (int i = 0 ; i < 50; i++)
  {
    if (robo.GetPos().DistanceTo(target) > 0.1)
      usleep(50000);
  }
  //cruisetoBias(pos.GetX(), pos.GetY(), 600, -10, 30);
  //robo.GotoXY(targetx,targety);
  //cruisetoBias(targetx,targety,650,-10,30);
  //robo.cruisetoBias(m_ball->GetX(),m_ball->GetY(), 650, -10, 30);
  double phi = robo.GetPhi().Get();
  double robotBallAngle;
  Position pos = robo.GetPos();
  ComputeLineAngle(pos.GetX(), pos.GetY(), ball.GetX(), ball.GetY(), &robotBallAngle);
  double diff1 = getDiffAngle(robotBallAngle, phi);
  double robotBallAngle2 = robotBallAngle <= 0 ? robotBallAngle + M_PI : robotBallAngle - M_PI;
  double diff2 = getDiffAngle(robotBallAngle2, phi);
  bool forward = fabs(diff1) < fabs(diff2);
  double a = forward ? robotBallAngle : robotBallAngle2;
  double diff3 = getDiffAngle(a, phi);
  int i;
  //cout << "start" << endl;
  for (i = 0 ; i < 1000 && fabs(diff3) >= 5 * M_PI / 180 ; i++)
  {
    if (diff3 > 0)
    {
      robo.MoveMs(50, -50, 500);
      //usleep(50000);
    }
    else if (diff3 < 0)
    {
      robo.MoveMs(-50, 50, 500);
      //usleep(50000);
    }
    phi = robo.GetPhi().Get();
    diff3 = getDiffAngle(a, phi);
  }
  //cout << "end1" << endl;
  robo.MoveMs(100, 100, 500);}
    else{ Position pos5=Position(1.4,0.0);
        double d = pos5.DistanceTo(ball.GetPos());
        double cosAngle, sinAngle;
        double targetx, targety;
        Position target = Position(targetx, targety);
        ComputeLineAngle(pos5.GetX(), pos5.GetY(), ball.GetX(), ball.GetY(), &cosAngle, &sinAngle);
        ComputeVectorEnd(pos5.GetX(), pos5.GetY(), cosAngle, sinAngle, d + 0.10, &targetx, &targety);
        robo.GotoXY(targetx, targety);
        for (int i = 0 ; i < 50; i++)
        {
          if (robo.GetPos().DistanceTo(target) > 0.1)
            usleep(50000);
        }
        //cruisetoBias(pos.GetX(), pos.GetY(), 600, -10, 30);
        //robo.GotoXY(targetx,targety);
        //cruisetoBias(targetx,targety,650,-10,30);
        //robo.cruisetoBias(m_ball->GetX(),m_ball->GetY(), 650, -10, 30);
        double phi = robo.GetPhi().Get();
        double robotBallAngle;
        Position pos = robo.GetPos();
        ComputeLineAngle(pos.GetX(), pos.GetY(), ball.GetX(), ball.GetY(), &robotBallAngle);
        double diff1 = getDiffAngle(robotBallAngle, phi);
        double robotBallAngle2 = robotBallAngle <= 0 ? robotBallAngle + M_PI : robotBallAngle - M_PI;
        double diff2 = getDiffAngle(robotBallAngle2, phi);
        bool forward = fabs(diff1) < fabs(diff2);
        double a = forward ? robotBallAngle : robotBallAngle2;
        double diff3 = getDiffAngle(a, phi);
        int i;
        //cout << "start" << endl;
        for (i = 0 ; i < 1000 && fabs(diff3) >= 5 * M_PI / 180 ; i++)
        {
          if (diff3 > 0)
          {
            robo.MoveMs(50, -50, 200);
            //usleep(50000);
          }
          else if (diff3 < 0)
          {
            robo.MoveMs(-50, 50, 200);
            //usleep(50000);
          }
          phi = robo.GetPhi().Get();
          diff3 = getDiffAngle(a, phi);
        }
        //cout << "end1" << endl;
        robo.MoveMs(50, 50, 200);}
  //usleep(50000);
 // cout << "end2" << endl;
}
