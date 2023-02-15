/** Sets all members of the MotionRequest representation for simple standing */
option(FightForBall, (const Pose2f&) opponent)
{
  Pose2f globbal = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y() );
  /** Set the motion request. */
  initial_state(start)
  {
    transition
    {
      goto goToBall;
    }
    action
    {
      
    }
  }
common_transition{
  
    if(theTeamBallModel.position.x() < theRobotPose.translation.x() - 10){
        goto goToBall;
    }
    
}
state(goToBall)
  {
    transition
    {
      if(theLibCodeRelease.angleToTarget(globbal.translation.x() , globbal.translation.y()) > Angle::fromDegrees(30.f)){
            goto turn;
      }
      if(theLibCodeRelease.distance(globbal, Pose2f(theRobotPose.translation)) < 400.f){
        if(theLibCodeRelease.distance(theLibCodeRelease.activeOpponent(),theTeamBallModel.position) < 1200.f &&
            std::abs(theLibCodeRelease.activeOpponent().translation.y() - theRobotPose.translation.y() ) <= 500.f 
            && theRobotPose.translation.x() < globbal.translation.x() ){
              goto dribble ;
        }
        else
        {
          goto walkWithBall;
        }
          
      }
      if(action_aborted){
        goto start;
      }
    }
    action
    {
      
        if(theTeamBallModel.position.x() < theRobotPose.translation.x()){
                // ChinaApproacher(true, Pose2f(4500,0)); 
                // included the function to pass 
                ChinaApproacher(true, theLibCodeRelease.poseToPass()); 
        }  
        else {
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(globbal.translation.x()-100.f,globbal.translation.y()) );
        }
    }
      
  }

  state(walkWithBall){
      transition{

          if(globbal.translation.x() < theRobotPose.translation.x() - 60){
            goto goToBall;
          }

          if(theLibCodeRelease.angleToTarget(globbal.translation.x() , globbal.translation.y()) > Angle::fromDegrees(30.f)){
            goto turn;
          }
          
          if(globbal.translation.x() > opponent.translation.x() +10 || 
            !theLibCodeRelease.between(globbal.translation.y(), opponent.translation.y()-250,opponent.translation.y()+250) ){
              
            if(theRobotPose.translation.x() > opponent.translation.x() +10){
              goto stolen;
            }
              
          }
      }action{
          if(theRobotPose.translation.y() > opponent.translation.y() ){
              RunUpWalkToTarget(Pose2f(1.f,1.f,1.f), Pose2f(0.f,theBallModel.estimate.position.x(),theBallModel.estimate.position.y()) + Pose2f(100.f,100.f));
          }else{
              RunUpWalkToTarget(Pose2f(1.f,1.f,1.f), Pose2f(0.f,theBallModel.estimate.position.x(),theBallModel.estimate.position.y()) + Pose2f(100.f,-100.f));
          }
          
      }
  }

  state(turn){
    transition{
      if(theLibCodeRelease.angleToTarget(globbal.translation.x() , globbal.translation.y()) < Angle::fromDegrees(25.f)){
        goto start;
      }
    }action{
      Turn(globbal);
    }
  }

  state(dribble)
  {
    transition
    {
      if( std::abs(theLibCodeRelease.activeOpponent().translation.y() - theRobotPose.translation.y() ) >= 300.f ||
          theBallModel.estimate.position.norm() > 700.f || state_time > 6000  ||
           theRobotPose.translation.x() > globbal.translation.x() -10) {
            goto goToBall;
      }              
    }
    action
    {
      Dribble();
    }
  }


  target_state(stolen)
  {
    transition
    {
      
    }
    action
    {
      
    }
  }

  
}
