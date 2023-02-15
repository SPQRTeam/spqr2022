#include "SPQRPlanner.h"

#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Utils/plannerSemaphore.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"



SPQRPlanner::SPQRPlanner(){
    SPQR::ConfigurationParameters();
}

float SPQRPlanner::definePassUtility(Stato s0, float parentWeight, float heuristic){
  float k1 = 2.1f, k2 = 0.1f, k3 = 0.1f, k4 = 0.5f;
  int k = 0;
  float min = 9000.f;
  float passUtility = 0.f;
  Pose2f myPose = theLibCodeRelease.disambiguateCell(s0.robotPose);

  for (k = 0; k < 16; k++){
    float dist = (float)(theLibCodeRelease.norm((4500.f - myPose.translation.x()),
                                                ((k*100 - 750) - myPose.translation.y())));

    if(min > dist){
      min = dist;
    }
  }
  passUtility = (9000 -min) * k1;


  passUtility += s0.freeGoalPercentage * k2;
  passUtility += s0.color * k3;
  passUtility += heuristic * k4;
  passUtility += parentWeight;

  return passUtility/100;

  return 2.0;


}

float myMin(float x1, float x2){
  if(x1 >= x2){
    return x2;
  }else{
    return x1;
  }
}


float myMax(float x1, float x2){
  if(x1 >= x2){
    return x1;
  }else{
    return x2;
  }
}

float SPQRPlanner::strikerUtility(Stato s0, float parentWeight, float heuristic){

  float k1 = 2.1f, k2 = 0.1f, k3 = 500.f, k4 = 0.5f;
  int k = 0;
  float min = 9000.f;
  float passUtility = 0.f;
  Pose2f myPose = theLibCodeRelease.disambiguateCell(s0.robotPose);
  int startLoop = -20;
  int endLoop = 20;
  float dist;


  if(startLoop < -30){
    startLoop = -30;
  }

  //std::cout<<s0.IHaveTheBall<<std::endl;
  if(s0.IHaveTheBall ||
    theLibCodeRelease.norm((s0.agreedBallPosition.x() - myPose.translation.x()),
     (s0.agreedBallPosition.y() -myPose.translation.y())) < 300){
    for (k = startLoop; k < endLoop; k++){
      float minX = 3800.f;
      //std::cout<<"min X"<<minX<<std::endl;
      dist =theLibCodeRelease.norm((minX - myPose.translation.x()),
        ((k*100) - myPose.translation.y()));



      if(min > dist){
        min = dist;
      }
    }
  }else{
    min = theLibCodeRelease.norm((s0.agreedBallPosition.x() - myPose.translation.x()),
     (s0.agreedBallPosition.y() -myPose.translation.y()));
  }
  //std::cout<<"abaco"<<std::endl;

  passUtility = (9000 -min) * k1;

  if(myPose.translation.x()> 3800.){
    passUtility -= 10000;
  }
  passUtility += s0.freeGoalPercentage * k2;
  passUtility += s0.color * k3;
  passUtility += heuristic * k4;
  passUtility += parentWeight;

  // float strikerDist = 9000;
  int i;


        //passUtility += k_fb* theLibCodeRelease.freeBallSight(myPose, s0.agreedBallPosition, s0.obstacles);


  return passUtility;

  return 1.0;

}




float SPQRPlanner::jollyUtility(Stato s0, float parentWeight, float heuristic){
  float k1 = 2.1f, k2 = 0.1f, k3 = 500.f, k4 = 0.5f;
  int k = 0;
  float min = 9000;
  float passUtility = 0;
  Pose2f myPose = theLibCodeRelease.disambiguateCell(s0.robotPose);
  float initY = (float)(s0.agreedBallPosition.y()/100);
  int startLoop = (int)myMax(-30.,initY - 5);
  int endLoop = (int)myMin(30., initY + 5);
  float dist;
  float ballDist = theLibCodeRelease.norm(s0.agreedBallPosition.x() - myPose.translation.x(),
    s0.agreedBallPosition.y() - myPose.translation.y());
  if(startLoop < -30){
    startLoop = -30;
  }
  //std::cout<<"abaco"<<std::endl;
  for (k = startLoop; k < endLoop; k++){
    float minX = myMin(s0.agreedBallPosition.x() + 900, 3500.);
    //std::cout<<"min X"<<minX<<std::endl;
    if(minX > 3000.){
      if(k >= -8 && k <= 8){

      }else{
        dist =theLibCodeRelease.norm((minX - myPose.translation.x()),
        ((k*100) - myPose.translation.y()));

      }
    }else{
      dist =theLibCodeRelease.norm((minX - myPose.translation.x()),
      ((k*100) - myPose.translation.y()));

    }

    if(min > dist){
      min = dist;
    }
  }
  passUtility = (9000 -min) * k1;

  if(myPose.translation.x()> 3800.){
    passUtility -= 10000;
  }
  passUtility += s0.freeGoalPercentage * k2;
  passUtility += s0.color * k3;
  passUtility += heuristic * k4;
  passUtility += parentWeight;
  passUtility += (1000 - ballDist);

  float strikerDist = 9000;
  unsigned i;
  for(i = 0; i < theTeamData.teammates.size(); i++){

    if(theTeamData.teammates.at(i).thePassShare.role == 5 || theTeamData.teammates.at(i).thePassShare.role == 6 ){
      strikerDist = theLibCodeRelease.norm(theTeamData.teammates.at(i).theRobotPose.translation.x() -
        myPose.translation.x(),
        theTeamData.teammates.at(i).theRobotPose.translation.y() -
        myPose.translation.y());

      if(theTeamData.teammates.at(i).theRobotPose.translation.x() <= myPose.translation.x() - 200){
        if(strikerDist < 2000){
          passUtility -= (4000 - strikerDist);
        }
      }else{
        if(strikerDist < 500){
          passUtility -= (1000 - strikerDist);
        }
      }
    }
  }

        //passUtility += k_fb* theLibCodeRelease.freeBallSight(myPose, s0.agreedBallPosition, s0.obstacles);


  return passUtility;

  return 1.0;

}



int SPQRPlanner::manhattanDistance(Stato as, Stato ps){

  if(as.robotPose == ps.robotPose){
    return 1;
  }else if(as.robotPose == ps.robotPose + 19 || as.robotPose == ps.robotPose + 17
    || as.robotPose == ps.robotPose - 19 || as.robotPose == ps.robotPose - 17){
    return 1;
  }else{
    return 2;
  }

}

Stato SPQRPlanner::completeState(Vector2f agreedBallPosition,int robotPose,float robotRotation,std::vector<Obstacle> obstacles, int action, float parentWeight, Stato prevState){
  bool IHaveTheBall;
  IHaveTheBall = theLibCodeRelease.otherStateHasBall(robotPose, agreedBallPosition);
  Pose2f statePose = theLibCodeRelease.disambiguateCell(robotPose);
  std::vector<float> freeAreas;
  float minDist = 9000;
  if(obstacles.size() > 0){
        freeAreas =  theLibCodeRelease.computeFreeAreas(statePose, obstacles);
        unsigned j;

        float adist;
        for(j = 0; j< obstacles.size(); j++){
          adist = theLibCodeRelease.norm(statePose.translation.x() - obstacles.at(j).center.x(),
            statePose.translation.y() - obstacles.at(j).center.y());
          if(adist < minDist ){
            minDist = adist;
          }
        }

    }else{
        freeAreas.push_back(730.);
        freeAreas.push_back(-730.);
    }

    std::vector<float> targetData =  theLibCodeRelease.computeTarget(freeAreas);
    Pose2f betterTarget = theLibCodeRelease.computeBetterTarget(targetData.at(0), theLibCodeRelease.disambiguateCell(robotPose));


    float freeGoalPercentage = targetData.at(1);

    Stato sr;
    sr.action = action;
    sr.freeGoalPercentage = freeGoalPercentage;
    sr.IHaveTheBall = IHaveTheBall;
    sr.betterTarget = betterTarget;
    sr.agreedBallPosition = agreedBallPosition;
    sr.robotPose = robotPose;
    sr.robotRotation = robotRotation;
    sr.obstacles = obstacles;

    if(minDist < 400){

          sr.color = 0;

        }else if(minDist >= 400 && minDist < 550){

          sr.color = 1;

        }else{
          sr.color = 2;
        }
    if((prevState.robotPose  % 18) > (sr.robotPose % 18)){
      sr.color = 0;
    }else if((prevState.robotPose  % 18) == (sr.robotPose % 18)){
      sr.color = 1;
    }

    float manH = manhattanDistance(sr,prevState);


  float strikerDist = 9000;
  if(thePassShare.role == 5){
    sr.utility = strikerUtility(sr,prevState.passUtil, 1/(float)manH);
  }else{
    sr.utility = jollyUtility(sr,prevState.passUtil, 1/(float)manH);
    unsigned i;
    for(i = 0; i < theTeamData.teammates.size(); i++){

      if(theTeamData.teammates.at(i).thePassShare.role == 5 || theTeamData.teammates.at(i).thePassShare.role == 6 ){
        strikerDist = theLibCodeRelease.norm(theTeamData.teammates.at(i).theRobotPose.translation.x() -
          statePose.translation.x(),
          theTeamData.teammates.at(i).theRobotPose.translation.y() -
          statePose.translation.y());
        if(theTeamData.teammates.at(i).theRobotPose.translation.x() < statePose.translation.x() + 150){
          if(strikerDist < 1000){
            sr.color = 0;
          }else if(strikerDist >= 1000 && strikerDist <= 1500){
            sr.color = 1;
          }else{
            sr.color = 2;
          }
        }

      }
    }


  }


    sr.passUtil = definePassUtility(sr,prevState.passUtil, 1/(float)manH);

    return sr;
}

bool SPQRPlanner::equalStates(Stato s1, Stato s2){
  if(s2.agreedBallPosition.x() != s1.agreedBallPosition.x()){

    return false;

  }

  if(s2.agreedBallPosition.y() != s1.agreedBallPosition.y()){

    return false;

  }

  if(s2.robotRotation != s1.robotRotation){

    return false;

  }

  if(s2.robotPose != s1.robotPose){

    return false;

  }

  return true;
}


Vector2f SPQRPlanner::newBallPose(int cell){
  Pose2f cellPose = theLibCodeRelease.disambiguateCell(cell);
  Vector2f newPose;
  newPose.x() = cellPose.translation.x();
  newPose.y() =cellPose.translation.y();
  return newPose;
}

std::vector<Stato> SPQRPlanner::exploitState( Stato sp){

    //std::vector<Stato> descendants;
    std::vector<Stato> descendants;
  //root della lista
  descendants.push_back(sp);


/* _ _ _
 *|   |   |   |
 *|+17|+18|+19|
 *|___|___|___|
 *|   |   |   |
 *| -1| X | +1|
 *|___|___|___|
 *|   |   |   |
 *|-19|-18|-17|
 *|___|___|___|
 */
  ////////////////////
  //Move descendants:
  ////////////////////

  if(sp.IHaveTheBall == false){

    //controllo prima colonna
    if(sp.robotPose % 18 == 0){
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

          Angle::fromDegrees(0), sp.obstacles,5,sp.utility, sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 19,

         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
      //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

          Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -17,

         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }else{

        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 19,

         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

          Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -17,

         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }
     //controllo ultima colonna
    }else if(sp.robotPose % 18 == 17){
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 17,

         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
      //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -19,

         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }else{

        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 19,

         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  +17,

         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }


    }else{
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  +17,

         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 19,

         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));


        //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 19,

         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -17,

         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));



      }else{
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 1,

         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 1,

         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  +17,

         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 18,

         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose + 19,

         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose - 19,

         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -18,

         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));
        descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose  -17,

         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));



      }

    }//fine move descendants
  }else{
    if(sp.robotPose % 18 == 0){
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 19),sp.robotPose + 19,
         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
      //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -17),sp.robotPose  -17,
         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }else{

        descendants.push_back(completeState(newBallPose(sp.robotPose + 18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 19),sp.robotPose + 19,
         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -17),sp.robotPose  -17,
         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }
     //controllo ultima colonna
    }else if(sp.robotPose % 18 == 17){
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose - 1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 17),sp.robotPose + 17,
         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
      //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose  -1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -19),sp.robotPose  -19,
         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }else{

        descendants.push_back(completeState(newBallPose(sp.robotPose +18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -19),sp.robotPose - 19,
         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose - 1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 17),sp.robotPose  +17,
         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));

      }


    }else{
      //controllo prima riga
      if(sp.robotPose < 18){
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose - 1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 17),sp.robotPose  +17,
         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 19),sp.robotPose + 19,
         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));


        //controllo ultima riga
      }else if(sp.robotPose > 197){
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose - 1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -19),sp.robotPose - 19,
         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -17),sp.robotPose  -17,
         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));



      }else{
        descendants.push_back(completeState(newBallPose(sp.robotPose -1),sp.robotPose - 1,
         Angle::fromDegrees(180), sp.obstacles,4,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 1),sp.robotPose + 1,
         Angle::fromDegrees(0), sp.obstacles,5,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 17),sp.robotPose  +17,
         Angle::fromDegrees(135), sp.obstacles,1,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 18),sp.robotPose + 18,
         Angle::fromDegrees(90), sp.obstacles,2,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose + 19),sp.robotPose + 19,
         Angle::fromDegrees(45), sp.obstacles,3,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -19),sp.robotPose - 19,
         Angle::fromDegrees(-135), sp.obstacles,6,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -18),sp.robotPose  -18,
         Angle::fromDegrees(-90), sp.obstacles,7,sp.utility,sp));
        descendants.push_back(completeState(newBallPose(sp.robotPose -17),sp.robotPose  -17,
         Angle::fromDegrees(-45), sp.obstacles,8,sp.utility,sp));



      }

    }//fine move descendants

  }

  descendants.push_back(completeState(sp.agreedBallPosition,sp.robotPose,

         Angle::fromDegrees(180), sp.obstacles,9,sp.utility,sp));
  return descendants;

}


void SPQRPlanner::update(PossiblePlan& pp) {
  //std::cout<<"armando"<<std::endl;
    std::vector<std::vector<Stato> > adjList;

  Stato s0;
  s0.action = 0;
  s0.robotPose = theSPQRInfoDWK.robotPose;
  s0.obstacles = theSPQRInfoDWK.obstacles;
  s0.freeGoalPercentage = theSPQRInfoDWK.freeGoalPercentage;
  s0.agreedBallPosition = theSPQRInfoDWK.agreedBallPosition;
  s0.IHaveTheBall = theSPQRInfoDWK.IHaveTheBall;
  s0.betterTarget = theSPQRInfoDWK.betterTarget;
  if(thePassShare.role == 5){
    s0.utility = strikerUtility(s0,0, 0);
  }else{
    s0.utility = jollyUtility(s0,0, 0);
  }

  s0.passUtil = definePassUtility(s0,0,0);

  //Pose2f myPose = theLibCodeRelease.disambiguateCell(s0.robotPose);

  //std::cout<<"freeBallSight "<<theLibCodeRelease.freeBallSight(myPose, s0.agreedBallPosition, s0.obstacles)<<std::endl;

  //std::cout<<"utility s0 = "<<s0.utility<<std::endl;

    std::vector<Stato> partialList;
  partialList = exploitState(s0);
  adjList.push_back(partialList);
  unsigned i;
  // int beg = 0;
  // int newStates = 0;
  // int newAdded = 0;


  float maxUtil = 0.f;

  ///TODO IMPLEMENTARE ALGORITMO PLANNING
  unsigned riga = 0;
  int best_target = 0;
  float maxPassUtil = 0.f;
  int selectedAction;

  //std::cout<<"adjListSize "<<adjList.size()<<std::endl;
  //std::cout<<"adjListSize 0 "<<adjList.at(0).size()<<std::endl;

  for(riga = 1; riga < adjList.at(0).size(); riga++){
    if(adjList.at(0).at(riga).utility > maxUtil ){
      maxUtil = adjList.at(0).at(riga).utility;
      selectedAction = adjList.at(0).at(riga).action;
    }
    if(adjList.at(0).at(riga).passUtil > maxPassUtil ){
      maxPassUtil = adjList.at(0).at(riga).passUtil;

    }
    //std::cout<<"utility riga "<<riga<<" = "<<adjList.at(0).at(riga).utility<<std::endl;
  }




    PossiblePlan::Action plan;
    int a;

    //std::cout<<"selectedAction "<<selectedAction<<std::endl;

  switch(selectedAction){
      //MOVE
      case 1: plan =   PossiblePlan::Move, a = s0.robotPose + 17; break;
      case 2: plan =   PossiblePlan::Move, a = s0.robotPose + 18; break;
      case 3: plan =   PossiblePlan::Move, a = s0.robotPose + 19; break;
      case 4: plan =   PossiblePlan::Move, a = s0.robotPose -1; break;
      case 5: plan =   PossiblePlan::Move, a = s0.robotPose + 1; break;
      case 6: plan =   PossiblePlan::Move, a = s0.robotPose -19; break;
      case 7: plan =   PossiblePlan::Move, a = s0.robotPose -18; break;
      case 8: plan =   PossiblePlan::Move, a = s0.robotPose - 17; break;
      case 9: plan =   PossiblePlan::Stand, a = -1; break;

      default : plan =   PossiblePlan::Stand, a = -1; break;

    }


    if (!s0.IHaveTheBall){

      //a = best_target;
      //plan =   PossiblePlan::Move;
    }

  pp.plan = plan;


  int movements = a;
  pp.movements = movements;
  pp.possibleTargets.clear();
  pp.betterTarget = theSPQRInfoDWK.betterTarget;

  pp.receiveUtil = maxPassUtil;

  //------debug next position and action -------
    DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");


    Pose2f center_temp = theLibCodeRelease.disambiguateCell(best_target);
    Pose2f centerRel_temp = theLibCodeRelease.glob2Rel(center_temp.translation.x(),center_temp.translation.y());
    CIRCLE("representation:ObstacleModel:rectangle" , centerRel_temp.translation.x(), centerRel_temp.translation.y(),
         150 , 20 ,  Drawings::dashedPen , ColorRGBA::black , Drawings::solidBrush , ColorRGBA::green  );


    Pose2f center = theLibCodeRelease.disambiguateCell(a);
    Pose2f centerRel = theLibCodeRelease.glob2Rel(center.translation.x(),center.translation.y());
    Pose2f actualPose = theLibCodeRelease.disambiguateCell(s0.robotPose);
    Pose2f myposeREL = theLibCodeRelease.glob2Rel(actualPose.translation.x(),actualPose.translation.y());


    CIRCLE("representation:ObstacleModel:rectangle" , centerRel.translation.x(), centerRel.translation.y(),
         150 , 20 ,  Drawings::dashedPen , ColorRGBA::black , Drawings::solidBrush , ColorRGBA::yellow  );

    CIRCLE("representation:ObstacleModel:rectangle" , myposeREL.translation.x(), myposeREL.translation.y(),
         150 , 20 ,  Drawings::dashedPen , ColorRGBA::black , Drawings::solidBrush , ColorRGBA::red  );



    if( plan ==  0 ) {
      DRAWTEXT("representation:ObstacleModel:rectangle" ,  myposeREL.translation.x(), myposeREL.translation.y()+1000 , 200 ,ColorRGBA::red , "Move");
    }
    else if( plan ==  3 ){
      DRAWTEXT("representation:ObstacleModel:rectangle" ,  myposeREL.translation.x(), myposeREL.translation.y()+1000 , 200 ,ColorRGBA::red , "Turn");
    }
    else if( plan ==  1 ){
      DRAWTEXT("representation:ObstacleModel:rectangle" ,  myposeREL.translation.x(), myposeREL.translation.y()+1000 , 200 ,ColorRGBA::red , "Turn");
    }
    else{
      DRAWTEXT("representation:ObstacleModel:rectangle" ,  myposeREL.translation.x(), myposeREL.translation.y()+1000 , 200 ,ColorRGBA::red , "ForwardKick");
    }


    for(i = 1; i < adjList.size(); i++){

      float ut = adjList.at(i).at(0).utility;
      Pose2f utilityPose = theLibCodeRelease.disambiguateCell(adjList.at(i).at(0).robotPose);

      //std::cout<<"utility cella  = "<<adjList.at(i).at(0).robotPose<<std::endl;

      Pose2f utilPoseRel = theLibCodeRelease.glob2Rel(utilityPose.translation.x(),utilityPose.translation.y());
      DRAWTEXT("representation:ObstacleModel:rectangle" ,  utilPoseRel.translation.x()-20, utilPoseRel.translation.y(),45 ,ColorRGBA::black , ut );


  }

}

MAKE_MODULE(SPQRPlanner, behaviorControl)
