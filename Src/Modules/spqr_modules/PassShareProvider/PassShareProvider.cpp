#include "PassShareProvider.h"

#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"


PassShareProvider::PassShareProvider(){
    SPQR::ConfigurationParameters();
}

/*
(float) sharedGoalUtil,
  (int) myNumber,
  (int) passingTo,
  (int) readyReceive,
  (int) readyPass,

*/


void PassShareProvider::update(PassShare& ps) {
  Pose2f passTarget = theLibCodeRelease.poseToPass();
  if(passTarget.translation.x() != theFieldDimensions.xPosOpponentGroundline &&
     passTarget.translation.y() != 0) {
    ps.passTarget = passTarget;
    ps.myNumber = theRobotInfo.number;
    ps.passingTo = theLibCodeRelease.isTargetToPass;
    ps.readyReceive = 1;
    ps.readyPass = 0;
    if( theBehaviorStatus.activity == BehaviorStatus::Activity::passing ){
      ps.readyPass = 1;
    }
  }
  // ps.sharedGoalUtil = thePossiblePlan.receiveUtil;

  // ps.myNumber = theRobotInfo.number;

  // if(theRole.role == Role::RoleType::goalie)
  //               ps.role = 1;
  //           else if(theRole.role== Role::RoleType::defender)
  //               ps.role = 2;
  //           else if(theRole.role== Role::RoleType::supporter)
  //               ps.role = 3;
  //           else if(theRole.role == Role::RoleType::jolly)
  //               ps.role = 4;
  //           else if(theRole.role == Role::RoleType::striker)
  //               ps.role = 5;
  //      else if(theRole.role == Role::RoleType::searcher_1)
  //               ps.role = 6;
  //           else if(theRole.role == Role::RoleType::searcher_2)
  //               ps.role = 7;
  //           else if(theRole.role == Role::RoleType::searcher_3)
  //               ps.role = 8;
  //           else if(theRole.role == Role::RoleType::searcher_4)
  //               ps.role = 9;


  // //std::cout<<"Numero "<<ps.myNumber<<" PassUtil "<<ps.sharedGoalUtil<<std::endl;
  // int i;

  // for(i = 0; i < theTeamData.teammates.size(); i++ ){
  //  if(theTeamData.teammates.at(i).thePassShare.sharedGoalUtil > (ps.sharedGoalUtil *1.1)&& theSPQRInfoDWK.IHaveTheBall == 1){
  //    ps.passingTo = theTeamData.teammates.at(i).thePassShare.myNumber;
  //    //std::cout<<"myUtil ="<<ps.sharedGoalUtil<<std::endl;
  //    //std::cout<<"opponent util = "<<theTeamData.teammates.at(i).thePassShare.sharedGoalUtil<<std::endl;

  //  }else{
  //    ps.passingTo = -1;
  //  }
  // }

  // ps.readyPass = 0;
  // if(ps.passingTo != -1){

  //  for(i = 0; i < theTeamData.teammates.size(); i++){
  //    if(theTeamData.teammates.at(i).thePassShare.myNumber == ps.passingTo){
  //      if(theTeamData.teammates.at(i).thePassShare.readyReceive == 1){
  //        ps.readyPass = 1;
  //      }
  //    }
  //  }

  // }

  // ps.readyReceive = 0;
  // for(i = 0; i < theTeamData.teammates.size(); i++){
  //  if(theTeamData.teammates.at(i).thePassShare.passingTo == ps.myNumber){

  //    Pose2f teammatePose = theTeamData.teammates.at(i).theRobotPose;
  //    if(std::abs(theLibCodeRelease.angleToTarget(teammatePose.translation.x(),
  //      teammatePose.translation.y())) < Angle::fromDegrees(5.f) ){
  //      ps.readyReceive = 1;
  //    }

  //  }
  // }



}

MAKE_MODULE(PassShareProvider, modeling)
