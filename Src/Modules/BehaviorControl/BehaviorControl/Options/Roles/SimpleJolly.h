/** A test striker option without common decision */


option(SimpleJolly)
{
  Pose2f strikerPos;
  float jollyYPos;
  float jollyXpos;
  bool  turnOnce;
  initial_state(start)
  {
    transition
    {
      strikerPos = Pose2f(-500,0);
      if(state_time > 10)
        goto follow;
    }
    action
    {
      lookLeftAndRight();
      Stand();
    }
  }

  common_transition
    {
        for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if(theTeamData.teammates.at(i).thePassShare.role == 5){

                  strikerPos = theTeamData.teammates.at(i).theRobotPose;


                }
            }
        if(strikerPos.translation.x() > -800){
          if(strikerPos.translation.x() < 0){
            jollyXpos = -400;
          }else{
            jollyXpos = std::min((strikerPos.translation.x() - 1000), 1000.f);
          }
          jollyXpos = std::min((strikerPos.translation.x() - 1000), 1000.f);
          if(strikerPos.translation.y() > 0){

            jollyYPos = strikerPos.translation.y() - 700;
          }else{
            jollyYPos = strikerPos.translation.y() + 700;

          }
        }else{
          jollyYPos = strikerPos.translation.y();
          if(strikerPos.translation.y() > 0){

            jollyYPos = strikerPos.translation.y() - 700;
          }else{
            jollyYPos = strikerPos.translation.y() + 700;

          }
        }
        if(jollyYPos > 2000){
          jollyYPos = 2000;
        }else if(jollyYPos < -2000){
          jollyYPos = -2000;
        }
    }

  state(follow){
    transition{

      if(theLibCodeRelease.norm((theRobotPose.translation.x() - jollyXpos ),
                                    (theRobotPose.translation.y() - jollyYPos)) < 450 ){
        turnOnce = true;
        goto turnToGlobalBall;
      }
    }action{
      lookLeftAndRight();
      WalkToTargetPathPlanner(Pose2f(0.9f,0.9f,0.9f), Pose2f(jollyXpos,jollyYPos));
      //Arms back
      theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
      theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
      theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
      theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
    }
  }

  state(turnToGlobalBall){
    transition{
      if(state_time > 10000 ||
        theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y() ) < Angle::fromDegrees(10.f)){
        goto wait;
      }
    }action{
      Turn(Pose2f(theTeamBallModel.position));

    }
  }



  state(wait){
    transition{
      if(state_time > 10000 || theLibCodeRelease.norm((theRobotPose.translation.x() - jollyXpos ),
                                    (theRobotPose.translation.y() - jollyYPos)) > 500){
        goto follow;
      }
    }action{

      Stand();
      lookAtBall();
    }
  }



}
