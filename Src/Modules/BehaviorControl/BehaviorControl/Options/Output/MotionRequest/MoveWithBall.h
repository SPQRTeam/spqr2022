#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

option(MoveWithBall, (const Pose2f&) target)
{   
  float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y());
  Pose2f relTarget = theLibCodeRelease.glob2Rel(target.translation.x(), target.translation.y());
  Pose2f ball_glob = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
  DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");

  LINE("representation:ObstacleModel:rectangle", 0, 0 , relTarget.translation.x(), relTarget.translation.y(), 20, Drawings::solidPen, ColorRGBA::black);

  initial_state(start)
  {
      transition
      {   

          goto turnToBall;
          
      }
      action
      {   
          Stand();
      }
  }
  state(turnToBall)
  {
      transition
      {
        if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f)){
           
            if((std::abs(theBallModel.estimate.position.angle())-std::abs(angle)) > Angle::fromDegrees(2.f)){
              goto walkAhead;
            }
            else if(theBallModel.estimate.position.norm() > 200.f){
              goto walkToBall;
            }else{
              goto alignBall;
            }
        }

      }
      action
      {   
          if(theBallModel.estimate.position.angle() < 0){
              WalkAtRelativeSpeed(Pose2f(-20.f,0.001f,0.001f));
          }else{
              WalkAtRelativeSpeed(Pose2f(20.f,0.001f,0.001f));
          }
      }
  }
  common_transition
  {
      if(theLibCodeRelease.discretizePose(theRobotPose.translation.x(),theRobotPose.translation.y()) ==

          theLibCodeRelease.discretizePose(target.translation.x(),target.translation.y())){
          goto targetState;
      }

      if(theBallModel.estimate.position.norm() >= 550 || thePossiblePlan.plan == PossiblePlan::ForwardKick){
        goto targetState;
      }

      if(thePassShare.readyPass == 1){
        goto targetState;
      }

      
  }
  state(alignBall)
  {
    transition
    {
      if(theBallModel.estimate.position.norm() > 300.f){
        goto walkToBall;
      }
      if(std::abs(angle) <= Angle::fromDegrees(5.f))  {
        std::cout<<angle<<std::endl;
        goto walkAhead;
      }
      if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(60.f)){
        goto turnToBall;
      }
    }
    action
    {
      //WalkToBallWithoutAlign();
      lookAtBall();
      if(angle < 0){
        WalkAtRelativeSpeed(Pose2f(-0.8f, 0.f, 0.8f));  
      }else{
        WalkAtRelativeSpeed(Pose2f(0.8f, 0.f, -0.8f));
      }
      
      
    }
  }
  state(walkAhead)
  {
    transition
    {
      if(theBallModel.estimate.position.norm() > 300.f){
        goto walkToBall;
      }
      if(std::abs(angle) > Angle::fromDegrees(10.f))  {
        goto alignBall;
      }
      if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(45.f)){
        goto turnToBall;
      }
    }
    action
    {
      
      WalkToTarget( Pose2f(0.01f, 0.7f, 0.7f), Pose2f(theBallModel.estimate.position + Vector2f(100.0f, .0f)) );

    
    }
  }
  state(walkToBall)
  {
    transition
    {
      if(theBallModel.estimate.position.norm() < 300.f){
        goto alignBall;
      }
      if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(45.f)){
        goto turnToBall;
      }
    }
    action
    {

      WalkToTarget( Pose2f(1.f, 1.f, 1.f), 
        Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
    }
  }
  target_state(targetState){

  }

/*
    initial_state(start)
    {
        transition
        {   

            if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f)){
                goto turnToBall;
            }else{
                goto walkToTarget;
            }
            
        }
        action
        {   

            //std::cout<<"1"<<std::endl;
            Stand();
        }
    }

    state(turnToBall)
    {
        transition
        {
            
            if(std::abs(theBallModel.estimate.position.norm()) > 550.f|| thePossiblePlan.plan == PossiblePlan::ForwardKick){

              goto targetState;
            }

            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f)){
               
                goto walkToBall;
            }

        }
        action
        {   
            if(theBallModel.estimate.position.angle() < 0){
                WalkAtRelativeSpeed(Pose2f(-20.f,0.001f,0.001f));
            }else{
                WalkAtRelativeSpeed(Pose2f(20.f,0.001f,0.001f));
            }
        }
    }

    state(walkToBall)
      {
        transition
        {

            if(std::abs(theBallModel.estimate.position.norm()) > 550.f || thePossiblePlan.plan == PossiblePlan::ForwardKick){

              goto targetState;
            }

          if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(5.f))
            goto turnToBall;
          if(theBallModel.estimate.position.norm() < 350.f)
            goto alignToTarget;
        }
        action
        {
          theHeadControlMode = HeadControl::lookForward;
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
        }
      }

    state(alignToTarget)
    {
      transition
      {

        if(std::abs(theBallModel.estimate.position.norm()) > 550.f || thePossiblePlan.plan == PossiblePlan::ForwardKick){

          goto targetState;
        }
        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y())) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
          goto alignBehindBall;
      }
      action
      {
        lookAtBall();
        WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y()),
                                                         theBallModel.estimate.position.x() - 250.f,       theBallModel.estimate.position.y()));
      }
    }

    state(alignBehindBall)
      {
        transition
        {
          
          if(std::abs(theBallModel.estimate.position.norm()) > 550.f || thePossiblePlan.plan == PossiblePlan::ForwardKick){

            goto targetState;
          }
          if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
          && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
          && std::abs(theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y())) < 3_deg){
            std::cout<<"entrato"<<std::endl;
            goto walkToTarget;

          }
            
        }
        action
        {
          theHeadControlMode = HeadControl::lookForward;
          WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y()),
                                                        theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
        }
      }

    state(walkToTarget)
    {
        transition
        {
            
            if(std::abs(theBallModel.estimate.position.norm()) > 550.f || thePossiblePlan.plan == PossiblePlan::ForwardKick){

              goto targetState;
            }
            if(theLibCodeRelease.discretizePose(theRobotPose.translation.x(),theRobotPose.translation.y()) == 

                theLibCodeRelease.discretizePose(target.translation.x(),target.translation.y())){
                goto targetState;
            }

        }
        action
        {   
            //std::cout<<target.translation.x()<<std::endl;
            WalkToTarget(Pose2f(0.5f,0.5f,0.5f), relTarget);
        }
    }

    
    target_state(targetState){

    }
    */

}
