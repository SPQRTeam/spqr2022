option(Striker2020)
{
  Pose2f globball = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
  Pose2f activeOpponent = theLibCodeRelease.activeOpponent();
  initial_state(wait_kickoff) {
      transition {
        if((theGameInfo.kickingTeam == Global::getSettings().teamNumber && state_time > 10) ||
        theLibCodeRelease.distance(theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()),Pose2f(0.f,0.f)) > 200
        || state_time >= 10000){
            goto start;
        }
      } action {
          Stand();
      }
  }// wait kickoff
  
  state(start)
    {
      transition
        {
            if(theBallModel.estimate.position.norm() > 500){
                goto goToBall;
            }
            
            //If else over different field positions
          if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnGroundline*0.7f){
              std::cout << "< -0.7 " << std::endl;
              
              if(theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 800.f){
                  if(theLibCodeRelease.between(activeOpponent.translation.y() , theRobotPose.translation.y() - 200.f, theRobotPose.translation.y() + 200.f )){
                      goto fight;
                  }else{
                      goto rinvia;
                  }

              }else{
                if(theFreeCorridors.maxThreshold < 800.f) {
                    Vector2f passTarget = theLibCodeRelease.poseToPass().translation;
                     // kick failed, attempt to pass if we have a quite good positioning of the teammate
                    if(passTarget != Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f) &&
                        passTarget.x() > theFieldDimensions.xPosOpponentGroundline*0.5f) {
                        goto pass;
                    }
                }else{
                    goto rinvia;
                }
            }
              
          }else if(theRobotPose.translation.x() >= theFieldDimensions.xPosOwnGroundline*0.7f 
          && theRobotPose.translation.x() < theFieldDimensions.xPosOpponentGroundline*0.15f){
              std::cout << "-0.7 - 0.65 " << std::endl;
              
              if(theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 800.f){
                  
                    goto fight;
                  
                }else{
                if(theFreeCorridors.maxThreshold < 800.f) {
                    Vector2f passTarget = theLibCodeRelease.poseToPass().translation;
                        // kick failed, attempt to pass if we have a quite good positioning of the teammate
                    if(passTarget != Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f) &&
                        passTarget.x() > theFieldDimensions.xPosOpponentGroundline*0.5f) {
                        goto pass;
                    }
                }else{
                    goto walkWithBall;
                }
            }
          }else if(theRobotPose.translation.x() >= theFieldDimensions.xPosOpponentGroundline*0.15f 
          && theRobotPose.translation.x() < theFieldDimensions.xPosOpponentGroundline*0.65f){
              std::cout << "0.15 - 0.65 " << std::endl;
              if(theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 800.f){
                    goto fight;
                  
                }else if(theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) > 800.f && theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 1000.f){
                    goto fastKick;
                }else{
                    goto kick;
                }
          
        }else if(theRobotPose.translation.x() >= theFieldDimensions.xPosOpponentGroundline*0.65f){
            std::cout << ">0.65 " << std::endl;
              
            if(theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 800.f){
                  
                if(theLibCodeRelease.between(activeOpponent.translation.y() , theRobotPose.translation.y() - 200.f, theRobotPose.translation.y() + 200.f )){
                      goto fight;
                  }else{
                      goto fastKick;
                  }
                  
                }else{
                    goto fastKick;
                }
        }
        goto fastKick;
    }
      action
        {
          Stand();
          lookLeftAndRight();
        }
    }//start

    state(fight){
        transition{
            if(theBallModel.estimate.position.norm() > 500.f || globball.translation.x() > activeOpponent.translation.x() + 20){
                goto start;
            }
        }action{
            Fight2020(activeOpponent);
        }
    }

    state(walkWithBall) {
        transition
        {
            if(theRobotPose.translation.x() >= theFieldDimensions.xPosOwnGroundline*0.15f || theBallModel.estimate.position.norm() > 600.f || theLibCodeRelease.distance(theRobotPose.translation,activeOpponent) < 500.f ){
                goto start;
            }
        }
        action
        {
            Approacher2020(1, thePossiblePlan.betterTarget);
            lookAtBall();
        }
    }//walkWithBall

    state(kick){
        transition{
            if(theBallModel.estimate.position.norm() > 600.f || (theLibCodeRelease.distance(activeOpponent.translation, theRobotPose.translation) < 400.f && activeOpponent.translation.x()-theRobotPose.translation.x() > -20.f)){
                goto start;
            }
                
        } action {
            Approacher2020(0, thePossiblePlan.betterTarget);
            lookAtBall();
        }
    }//kick

    state(fastKick){
        transition{
            if(theBallModel.estimate.position.norm() > 600.f )
                goto start;
        } action {
            Approacher2020(2, thePossiblePlan.betterTarget);
            lookAtBall();
        }
    }//fastKick

    state(rinvia){
    transition{
        if(theBallModel.estimate.position.norm() > 600.f )
            goto start;
      } action {
          Approacher2020(2, Pose2f(theFieldDimensions.xPosOpponentGroundline, theRobotPose.translation.y()));
          lookAtBall();
      }
    }//rinvia

    state(goToBall) {
        transition
        {
            if(theBallModel.estimate.position.norm() < 450.f){
                goto start;
                
            }
        }
        action
        {
            Approacher2020(1, thePossiblePlan.betterTarget);
            lookLeftAndRight();
        }
    }//gotoBall

    state(pass)
    {
      transition
        {
          if(theBallModel.estimate.position.norm() > 600.f || theLibCodeRelease.poseToPass().translation.x() > theFieldDimensions.xPosOpponentGroundline || (theLibCodeRelease.distance(activeOpponent.translation, theRobotPose.translation) < 400.f && activeOpponent.translation.x()-theRobotPose.translation.x() > -20.f))
            goto start;
        }
      action
        {
          Activity( BehaviorStatus::Activity::passing );   // for passShareProvider
          Approacher2020(2, theLibCodeRelease.poseToPass());
          lookAtBall();
        }
    }//pass
}
