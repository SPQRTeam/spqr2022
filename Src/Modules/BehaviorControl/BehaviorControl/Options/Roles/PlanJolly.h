/** A test striker option without common decision */


///TODO Capire perché entrambi vanno all'indietro

#include <iostream>
bool justEnter_J;
float targetX_J = -10000;
float targetY_J = -10000;

Pose2f moveTarget_J;
Pose2f turnTarget_J;
Pose2f passingMate_J;
Pose2f passTarget_J;

option(PlanJolly)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto generalState;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
      justEnter_J = true;
      
    }
  }
  common_transition
  {
          theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
          theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
          theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
          theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;


  }
  state(generalState)
  {
    transition
    {

      int i;
      for(i = 0; i < theTeamData.teammates.size(); i++){
        if(theTeamData.teammates.at(i).thePassShare.passingTo == thePassShare.myNumber){
          passingMate_J = theTeamData.teammates.at(i).theRobotPose;
          goto receivePass;
        }
        if(theTeamData.teammates.at(i).thePassShare.myNumber == thePassShare.passingTo){
          if(theTeamData.teammates.at(i).thePassShare.readyReceive == 1){
            passingMate_J = theTeamData.teammates.at(i).theRobotPose;
            goto executePass;
          }
        }
      }
      switch(thePossiblePlan.plan){
        case PossiblePlan::Move: goto Move;   break;
        case PossiblePlan::Stand: goto Stand; break;
        case PossiblePlan::ForwardKick: goto Stand; break; 
        case PossiblePlan::Turn: goto turn; break;
        
      }
    }
    action
    { 
      
        if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
            Stand();
            lookAtBall();
        }
        else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
            Stand();
            lookAtGlobalBall();
        }
        else{
            Stand();
            lookLeftAndRight();
        }

        justEnter_J = true;
        targetX_J = -10000;
        targetY_J = -10000;
    }
  }
  

  state(Move)
  {
    transition
    {
      if(state_time > 4000 || (state_time > 10 && action_done))
        goto generalState;
    }
    action
    { 
      //std::cout<<planIndex<<std::endl;
      
      if(justEnter_J == true){
        
        moveTarget_J = theLibCodeRelease.disambiguateCell(thePossiblePlan.movements);
        
        
        justEnter_J = false;
      }
      lookLeftAndRight();

      Move(moveTarget_J);
    }
  }

 
  state(MoveWithBall){
    transition
    {
      
      if(state_time > 10000 ||(state_time > 10 && action_done)){

        //std::cout<<"so uscito "<<state_time<<std::endl;
        goto generalState;

      }
        
    }
    action
    { 
      //std::cout<<planIndex<<std::endl;
      
       
      if(justEnter_J == true){
        
        moveTarget_J = theLibCodeRelease.disambiguateCell(thePossiblePlan.movements);
        
        
        justEnter_J = false;
      }
      theHeadControlMode = HeadControl::lookForward;
      MoveWithBall(moveTarget_J);

    }


  }

  
  state(Stand){
    transition{
      if(state_time >= 2000){
        goto generalState;
      }
    }action{
      

      lookLeftAndRight();

      if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < (2000)){
        lookAtBall();
        turnTarget_J = (theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()));
      } else{
        turnTarget_J = (Pose2f(theTeamBallModel.position.x(),theTeamBallModel.position.y())); 
        
      }
      
      if(std::abs(theLibCodeRelease.angleToTarget(turnTarget_J.translation.x(), turnTarget_J.translation.y()))
       >= Angle::fromDegrees(15.f)){
        Turn(turnTarget_J);
      }else{
        Stand();
        lookLeftAndRight();
      }


    }
  }

  state(forwardKick)
  {
    transition
    {
      if(state_time > 8000 || (state_time > 10 && action_done))
        goto generalState;
    }
    action
    {
      
      
    }
  }

  state(turn){

    transition
    {
      if(state_time > 4000 || (state_time > 10 && action_done))
        goto generalState;
    }
    action
    {
      if(justEnter_J == true){
        
        turnTarget_J = theLibCodeRelease.disambiguateCell(thePossiblePlan.movements);
        
        
        justEnter_J = false;
      }
      theHeadControlMode = HeadControl::lookForward;
      Turn(turnTarget_J);
    }

  }

  state(receivePass){
    transition
    {
      if(state_time > 4000 || (state_time > 10 && action_done))
        goto generalState;
    }
    action
    {
      if(justEnter_J == true){
        
        turnTarget_J = passingMate_J;
        
        
        justEnter_J = false;
      }
      theHeadControlMode = HeadControl::lookForward;
      Turn(turnTarget_J);
    }


  }

  state(executePass){
    transition
    {
      if(state_time > 6000 || (state_time > 10 && action_done))
        goto generalState;
    }
    action
    {
      if(justEnter_J == true){
        
        passTarget_J = passingMate_J;
        
        
        justEnter_J = false;
      }
      srand (time(NULL));
      theHeadControlMode = HeadControl::lookForward;
      
      
        //std::cout<<"eseguo kicktotarget"<<std::endl;
      //std::cout<<"pt x = "<<thePossiblePlan.betterTarget.translation.x()<<" pt y = "<<thePossiblePlan.betterTarget.translation.y()<<std::endl;
      
      Approacher(true, passTarget_J);
      
    }
  }
}
