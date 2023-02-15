/** Sets all members of the MotionRequest representation for executing a TargetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, in percentage.
 *  @param target Walking target, in mm and radians, absolute to the robot.
 *   (float)(500.f) uprightRobotRadius, Radius to walk around an upright robot (in mm).
 *   (float)(550.f) fallenRobotRadius, Radius to walk around a fallen robot (in mm).
 *   (float)(750.f) readyRobotRadius,  Radius to walk around a robot in ready state (in mm).
 */
#include "Representations/Modeling/TeamPlayersModel.h"
#define FREEDIRECTION_OFF

option(SPQRWalkTo, (const Pose2f&) speed, (const Pose2f&) target, (const float) angle)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
    }
    action
    {
      float x1 = theRobotPose.translation.x();
      float x2 = target.translation.x(); //ballGlob.translation.x();
      float y1 = theRobotPose.translation.y();
      float y2 = target.translation.y();  //ballGlob.translation.y();
      float m = (y1-y2)/(x1-x2) ;
      float q = y1 - (((y1-y2)/(x1-x2))*x1) ;
      bool freeDirection = true ;
      float maxrange =  (x2<x1)? x1:x2; // (targetX <robotX )? robotX:targetX;  portion of area in wich obstacle are considered
      float minrange =  (x2>x1)? x1:x2;
      float targetDistance = std::sqrt( std::abs(x1-x2)*std::abs(x1-x2)+std::abs(y1-y2)*std::abs(y1-y2) ) ;
      for(const auto& obstacle : theObstacleModel.obstacles){
        if(obstacle.type != 0 ){       //do not consider goalpost
          Pose2f obstacleGlob =  theLibCodeRelease.rel2Glob(obstacle.center.x(),obstacle.center.y());
          float x3 = obstacleGlob.translation.x();
          float y3 = obstacleGlob.translation.y();
          float distance = std::abs( y3 - (m*x3 +q) )/(std::sqrt( 1 + (m*m) ));
          if( distance <  500.f &&  maxrange >= x3 &&  x3>= minrange && obstacle.type != 0){
              freeDirection = false;
          }
        }

      }
#ifdef FREEDIRECTION_OFF
      if( x2 > x1  && std::abs(x2-x1)< 700.f && std::abs(y2-y1)< 300.f && std::abs(angle) < 90_deg ){
#endif
#ifndef FREEDIRECTION_OFF
      if(freeDirection == true  && x2 > x1  && std::abs(x2-x1)< 700.f && std::abs(y2-y1)< 300.f && std::abs(angle) < 90_deg ){
#endif
        // std::cout<<"senza pathplanner con distanzaX: "<<std::abs(x1-x2)<<std::endl;
        // std::cout<<"senza pathplanner con angolo: "<<std::abs(angle)<<std::endl;
        Pose2f relTarget = theLibCodeRelease.glob2Rel(target.translation.x(), target.translation.y());
//        Pose2f angleTarget = Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() -170.f ,theBallModel.estimate.position.y() -40);
        theMotionRequest.motion = MotionRequest::walk;
        theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
        theMotionRequest.walkRequest.target = relTarget;
        theMotionRequest.walkRequest.speed = speed;
        theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      }
      else{
        // std::cout<<"pathplanner : "<<std::abs(x1-x2)<<std::endl;
        MotionRequest mr = thePathPlanner.plan(target,speed,false);
        assert(speed.translation.x()!=0.f && speed.translation.y()!=0.f && speed.rotation!=0.f);
        theMotionRequest.motion = mr.motion;
        theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
        theMotionRequest.walkRequest.target = mr.walkRequest.target;
        theMotionRequest.walkRequest.speed = mr.walkRequest.speed;
        theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      }

    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
    }
    action
    {
      Pose2f ballGlob =  theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x() , theBallModel.estimate.position.y()) ;
      float x1 = theRobotPose.translation.x();
      float x2 = ballGlob.translation.x();
      float y1 = theRobotPose.translation.y();
      float y2 = ballGlob.translation.y();
      float m = (y1-y2)/(x1-x2) ;
      float q = y1 - (((y1-y2)/(x1-x2))*x1) ;
      bool freeDirection = true ;
      float maxrange =  (x2<x1)? x1:x2;
      float minrange =  (x2>x1)? x1:x2;
      float targetDistance = std::sqrt( std::abs(x1-x2)*std::abs(x1-x2)+std::abs(y1-y2)*std::abs(y1-y2) ) ;
      for(const auto& obstacle : theObstacleModel.obstacles){
        if(obstacle.type != 0 ){
          Pose2f obstacleGlob =  theLibCodeRelease.rel2Glob(obstacle.center.x(),obstacle.center.y());
          float x3 = obstacleGlob.translation.x();
          float y3 = obstacleGlob.translation.y();
          float distance = std::abs( y3 - (m*x3 +q) )/(std::sqrt( 1 + (m*m) ));
          if( distance <  500.f &&  maxrange >= x3 &&  x3>= minrange && obstacle.type != 0){
              freeDirection = false;
          }
        }

      }
      // freeDirection = false;
#ifdef FREEDIRECTION_OFF
      if( x2 > x1  && std::abs(x2-x1)< 700.f && std::abs(y2-y1)< 300.f && std::abs(angle) < 90_deg ){
#endif
#ifndef FREEDIRECTION_OFF
      if(freeDirection == true  && x2 > x1  && std::abs(x2-x1)< 700.f && std::abs(y2-y1)< 300.f && std::abs(angle) < 90_deg ){
#endif
        // std::cout<<"senza pathplanner con distanzaX: "<<std::abs(x1-x2)<<std::endl;
        // std::cout<<"senza pathplanner con angolo: "<<std::abs(angle)<<" --- "<<90_deg<<std::endl;
        Pose2f relTarget = theLibCodeRelease.glob2Rel(target.translation.x(), target.translation.y());
        // angle come parametro da problemi- reimpostato theLibCodeRelease.angleToGoal
//        Pose2f angleTarget = Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() -150.f ,theBallModel.estimate.position.y() -60);
        theMotionRequest.motion = MotionRequest::walk;
        theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
        theMotionRequest.walkRequest.target = relTarget;
        theMotionRequest.walkRequest.speed = speed;
        theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      }
      else{
          // std::cout<<"pathplanner2 : "<<std::abs(x1-x2)<<std::endl;
          MotionRequest mr = thePathPlanner.plan(target,speed,false);
          assert(speed.translation.x()!=0.f && speed.translation.y()!=0.f && speed.rotation!=0.f);
          theMotionRequest.motion = mr.motion;
          theMotionRequest.walkRequest.mode = mr.walkRequest.mode;
          theMotionRequest.walkRequest.target = mr.walkRequest.target;
          theMotionRequest.walkRequest.speed = mr.walkRequest.speed;
          theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
      }

    }
  }
}
