option(StrikerAUMF)
{
  Pose2f globball = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
  Pose2f activeOpponent = theLibCodeRelease.activeOpponent();
  bool first = true;
  bool firstWalk = true; initial_state(wait_kickoff) {transition {
          if((theGameInfo.kickingTeam == Global::getSettings().teamNumber && state_time > 10) ||
         theLibCodeRelease.distance(theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()),Pose2f(0.f,0.f)) > 200
         || state_time >= 10000){
             goto start;
         }
      } action {
          Stand();
      }
  }
  state(start)
    {
      transition
        {
          // try to pass if to close to our goal
          if(theRobotPose.translation.x()  < theFieldDimensions.xPosOwnGroundline*0.6f){
            if(theLibCodeRelease.poseToPass().translation != Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)){
              goto pass;
            }
          } else if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentGroundline*0.25f) {
            // if close to opponent goal
            // check if we have space and get the ratio w.r.t. the distance
            // (basically weight the free goal area and the distance. The closer the less free area needed)
            Pose2f BGA = theLibCodeRelease.biggestFreeArea(theLibCodeRelease.computeFreeAreas(Pose2f(theRobotPose.translation), theTeamPlayersModel.obstacles));
            float bgadist = std::abs(BGA.translation.x() - BGA.translation.y());
            float ratio = theLibCodeRelease.distance(Pose2f(theRobotPose.translation), Pose2f(theFieldDimensions.xPosOpponentGroundline, 0.f))/bgadist;
            if(ratio<3) {
              // don't try to kick if too close to the sidelines
              if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline-300.f){
                goto kick;
              }
            } else if(theFreeCorridors.maxThreshold < 800.f) {
              Vector2f passTarget = theLibCodeRelease.poseToPass().translation;
              // kick failed, attempt to pass if we have a quite good positioning of the teammate
              if(passTarget != Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f) &&
                 passTarget.x() > theFieldDimensions.xPosOpponentGroundline*0.5f) {
                goto pass;
              }
            }
          } else if(theRobotPose.translation.x() >= theFieldDimensions.xPosOpponentGroundline*0.25f){
            // if we are far ahead then cycle over kick, ball keeping and fight
            if( theRoleAndContext.ball_holding_Context == 1){
              goto kick;
            }
            else if( theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 0 ){
              goto goToBall;
            }
            else if(theRoleAndContext.ball_holding_Context == 3 &&
                    activeOpponent.translation.x() > theRobotPose.translation.x() -20.f &&
                    globball.translation.x() > theRobotPose.translation.x()){
              goto fight;
            }
          }
          // if all failed carry on
          goto goToBall;
        }
      action
        {
          first = true;
          firstWalk = true;
        }
    }

  state(fight)
    {
      transition{
        firstWalk = true;
        if(globball.translation.x() > activeOpponent.translation.x()+10.f ||
           !theLibCodeRelease.between(globball.translation.y(), activeOpponent.translation.y()-250.f, activeOpponent.translation.y()+250.f)){
          if(theRobotPose.translation.x() > activeOpponent.translation.x()+20.f) {
            goto walkWithBall;
          }
        }
        if(theLibCodeRelease.distance(activeOpponent, Pose2f(theRobotPose.translation)) > 1600.f || theBallModel.estimate.position.norm() > 600.f) {
          goto goToBall;
        }
        if(theRobotPose.translation.x() >=  theFieldDimensions.xPosOpponentGroundline*0.25f) {
          if(theRoleAndContext.ball_holding_Context == 1) {
            if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline-300.f) {
              goto kick;
            } else {
              goto goToBall;
            }
          }
          else if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 0.f ){
            goto goToBall;
          }
        }
      }
      action
        {
          if(theRoleAndContext.atk_def_Context == 4){
            //FightForBall(activeOpponent);
            Fight2020(activeOpponent);
          } else {
            //FightForBall(Pose2f(theRobotPose.translation)+Pose2f(300.f,0.f));
            Fight2020(Pose2f(theRobotPose.translation)+Pose2f(300.f,0.f));
          }
          lookAtBall();
        }
    }//fight

  state(goToBall)
    {
      transition
        {
          firstWalk = true;
          if(theRoleAndContext.ball_holding_Context == 3 &&
           activeOpponent.translation.x() > theRobotPose.translation.x()-20.f &&
             globball.translation.x() > theRobotPose.translation.x()) {
            goto fight;
          } else if(theRobotPose.translation.x() >=  theFieldDimensions.xPosOpponentGroundline*0.55f) {
            if( theRoleAndContext.ball_holding_Context == 1){
              if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline - 300.f) {
                goto kick;
              }
            }
            else if(theRoleAndContext.ball_holding_Context == 3 &&
                    activeOpponent.translation.x() > theRobotPose.translation.x()-20.f &&
                    globball.translation.x() > theRobotPose.translation.x()){
              goto fight;
            }
          }

          float angle = theLibCodeRelease.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0.f);
          if(theRobotPose.translation.x() < theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()).translation.x()) {
            if(theLibCodeRelease.distance(theRobotPose.translation, theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y())) < 600.f) {
              if(std::abs(angle) <= Angle::fromDegrees(15.f)) {
                goto walkWithBall;
              }
            }
          }
    }
    action
      {
        if(globball.translation.x() < theRobotPose.translation.x()) {
          // included the function to pass
          ChinaApproacher(false, Pose2f(theFieldDimensions.xPosOpponentGroundline, 0.f));
        }
        else {
          WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(globball.translation.x()-100.f,globball.translation.y()));
        }
        lookAtBall();
      }
    }//gotoBall

  state(walkWithBall) {
    transition
      {
        if(theRobotPose.translation.x() < -theFieldDimensions.xPosOpponentGroundline * 0.6f){if(theLibCodeRelease.poseToPass().translation != Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f && theFreeCorridors.maxThreshold < 800.f)){
            goto pass;
          }
        } else if(theRobotPose.translation.x()  > theFieldDimensions.xPosOpponentGroundline * 0.25f) {
          Pose2f BGA = theLibCodeRelease.biggestFreeArea(theLibCodeRelease.computeFreeAreas(Pose2f(theRobotPose.translation), theTeamPlayersModel.obstacles));
          float bgadist = std::abs(BGA.translation.x() - BGA.translation.y());
          float ratio = theLibCodeRelease.distance(Pose2f(theRobotPose.translation), Pose2f(theFieldDimensions.xPosOpponentGroundline, 0))/ bgadist;
          if(ratio < 3 && theRobotPose.translation.x() > 750.f){
            if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftSideline - 300.f) {
              goto kick;
            } else {
              goto goToBall;
            }
          }

          if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation), activeOpponent) > 1000.f){
            if(theFreeCorridors.maxThreshold < 800.f && theLibCodeRelease.poseToPass().translation.x() < theFieldDimensions.xPosOpponentGroundline) {
              goto pass;
            }
          }
        }

        if(theRobotPose.translation.x() >=  theFieldDimensions.xPosOpponentGroundline*0.25){
          if( theRoleAndContext.ball_holding_Context == 1){
            goto kick;
          }
          else if( theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 0){
            goto goToBall;
          }
          else if(theRoleAndContext.ball_holding_Context == 3 &&
                  activeOpponent.translation.x() > theRobotPose.translation.x()-20.f &&
                  globball.translation.x() > theRobotPose.translation.x()){
            goto fight;
          }
        }
        if(theRoleAndContext.ball_holding_Context == 3 &&
           activeOpponent.translation.x() > theRobotPose.translation.x()-20.f &&
           globball.translation.x() > theRobotPose.translation.x()){
          goto fight;
        }

        if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation), globball) > 510.f){
          goto goToBall;
        }

        if(theRobotPose.translation.x() < theFieldDimensions.xPosOpponentGroundline*0.4 && theLibCodeRelease.poseToPass().translation.x() < theFieldDimensions.xPosOpponentGroundline && theLibCodeRelease.distance(globball, theLibCodeRelease.poseToPass()) > 2200.f && theLibCodeRelease.poseToPass().translation.x()-globball.translation.x() > 1200.f){

          if(theLibCodeRelease.between(theLibCodeRelease.distance(activeOpponent.translation, globball),  400.f, 1500.f) || activeOpponent.translation.x()-globball.translation.x() < 0.f){//   && theLibCodeRelease.poseToPass().rotation >5)
            goto pass;
          }
        }
      }
    action
      {
        Pose2f WalkTarget;
        if(firstWalk == true){
          if(std::abs(theRobotPose.translation.y()) > 2500.f){
            WalkTarget =  Pose2f((float)theRobotPose.translation.x(), 0.f);
          }
          WalkTarget =  Pose2f((float)theFieldDimensions.xPosOpponentGroundline, 0.f);
          firstWalk = false;
        }
        //WalkWithBall(Pose2f(1.f,1.f,1.f), WalkTarget);
        Approacher2020(1,WalkTarget);
        lookAtBall();
      }
  }//walkWithBall

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
          //ChinaApproacher(true, theLibCodeRelease.poseToPass());
          Approacher2020(2,theLibCodeRelease.poseToPass());
          lookAtBall();
        }
    }

  state(kick){
    transition{
      firstWalk = true;
      if(state_time > 5000.f) {
        goto start;
      }
      if(theBallModel.estimate.velocity.x() > 100.f) {
        goto start;

        if(theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < theRobotPose.translation.x()-60.f){
          goto start;
        }
        if(theBallModel.estimate.position.norm() > 600.f) {
          goto start;
        }
      }
      } action {
        Pose2f myTarget;
        if(first == true){
          myTarget =  thePossiblePlan.betterTarget;
          first = false;
        }
        if(theLibCodeRelease.distance(activeOpponent, Pose2f(theRobotPose.translation)) > 300.f){
          if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentGroundline - 500.f){
            if(std::abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftGoal){
              //WalkWithBall(Pose2f(1.f,1.f,1.f), Pose2f(theFieldDimensions.xPosOpponentGroundline + 2000.f, 0.f ));
              Approacher2020(1,Pose2f(theFieldDimensions.xPosOpponentGroundline + 2000.f, 0.f ));
            }
          }
        }

        // if your are in the center field use the china approachere to be more accurate
        // else you might want to kick very fast
        /* if(std::abs(theRobotPose.translation.x()) < theFieldDimensions.xPosOpponentGroundline*0.4 ) { */
        /*   ChinaApproacher(true, myTarget); */
        /* } else { */
        if(std::abs(theRobotPose.translation.x()) < theFieldDimensions.xPosOpponentGroundline*0.6 ) {
          
          //FastApproacher(theLibCodeRelease.glob2Rel(myTarget.translation.x(),myTarget.translation.y()).translation, false);
          if(theLibCodeRelease.distance(theRobotPose,activeOpponent) < 600){
            Approacher2020(2,myTarget);
          }else{
            Approacher2020(0,myTarget);
          }
          
        } else {
          //FastApproacher(theLibCodeRelease.glob2Rel(myTarget.translation.x(),myTarget.translation.y()).translation, true);
          Approacher2020(2,myTarget);
        }
        /* } */
      }
    }//kick
  }
