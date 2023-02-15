//#define SPQR_DEBUG_GOALIE

//#define SPQR_GOALIE_STAND


#ifdef PENALTY_STRIKER_GOALIE
#define SPQR_GOALIE_STAND
#endif

option(Goalie)
{
    common_transition
    {
        if((int)theGameInfo.setPlay == GOALFREEKICK)
            goto goalieKickAway;

        Vector2f velocity = theBallModel.estimate.velocity;
        Vector2f position = theBallModel.estimate.position;
#ifdef SPQR_GOALIE_STAND
      if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){
#else
      if(theGameInfo.state == STATE_PLAYING  && velocity.x() < 100 && velocity.norm()/position.norm()>1.8){ //& theBallModel.estimate.position.x() < 1500){
#endif
//          std::cout<<"velocita common  "<<velocity.norm()<<std::endl;
//          std::cout<<"posizione  common "<<position.norm()<<std::endl;
//          std::cout<<"soglia  common     "<<velocity.norm()/position.norm()<<std::endl;
          double teta = atan(velocity.y()/velocity.x());
            //std::cout<< "teta   "<<teta<<std::endl;
            float l = position.x()* (float) tan(teta);
            float lato = position.y()-l;
            //std::cout<< "lato  "<< lato <<std::endl;

            if(lato<=200 && lato >= -200)
                goto stopBall;
            else if(lato>200 && lato < 1500)
                goto goalieDiveLeft;
            else if(lato<-200 && lato>-1500)
                goto goalieDiveRight;

        }
    }




    initial_state(start)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            goto mainLoop;
#endif

            //theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KEEPER;
            if( !theLibCodeRelease.isGoalieInStartingPosition )
                goto gotoGoaliePosition;

            if(state_time > 500)
                goto turnToOpponentGoal;
        }
        action
        {
            Stand();
            lookAtBall();
        }
    }

    state(gotoGoaliePosition) // control conditions, too restrictive
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "gotogoalieposition" << std::endl;
#endif
            if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                    && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                    //&& theLibCodeRelease.isBallInArea      //   -4500 < x < -3900  &&  -1100 < y <1100
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)
                //&& theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )

                goto goalieKickAway;

            if(theLibCodeRelease.timeSinceBallWasSeen < 800){
                Vector2f velocity = theBallModel.estimate.velocity;
                Vector2f position = theBallModel.estimate.position;
                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), -4200.f + theLibCodeRelease.goalie_displacement,
                                                  SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y,
                                                      SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                goto turnToOpponentGoal;

            if( theLibCodeRelease.isGoalieInAngle )
                goto backTrackInPose;


        }
        action
        {
            //lookAtLandmark();
            if(std::abs(theLibCodeRelease.angleToTarget(-4200.f + theLibCodeRelease.goalie_displacement,
                                                     SPQR::GOALIE_BASE_POSITION_Y)) > 10.f/*Angle::fromDegrees(10.f)*/)
            {

                lookLeftAndRight();

                WalkToTarget(Pose2f(50.f, 50.f, 50.f),
                         Pose2f(theLibCodeRelease.angleToTarget(-4200.f +theLibCodeRelease.goalie_displacement,
                                                             SPQR::GOALIE_BASE_POSITION_Y), 0.f, 0.f));
            }
            else
            {
                lookLeftAndRight();

                WalkToTarget(Pose2f(50.f,50.f,50.f),
                          theLibCodeRelease.glob2Rel(-4200.f +theLibCodeRelease.goalie_displacement,
                                                 SPQR::GOALIE_BASE_POSITION_Y));
            }
        }

    }

    state(backTrackInPose)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrackinpose" << std::endl;
#endif

            if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                    && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                    //&& theLibCodeRelease.isBallInArea      //   -4500 < x < -3900  &&  -1100 < y <1100
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)
                //&& theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )

                goto goalieKickAway;


            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;


          if(theLibCodeRelease.timeSinceBallWasSeen < 800){

            Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

            if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                goto goalieOutOfPoles;


          }
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack balanced" <<  (theLibCodeRelease.isValueBalanced(theRobotPose.translation.x, -4200.f, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                                                   theLibCodeRelease.isValueBalanced(theRobotPose.translation.y, SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE)) << std::endl;
#endif
            if( theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), -4200.f, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
               //~ if( theLibCodeRelease.isGoalieInAngle() )
            goto mainLoop;
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack " << theLibCodeRelease.norm(theRobotPose.translation.x() - -4200.f,
                                                                theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) << std::endl;
#endif
#ifndef SPQR_GOALIE_NOT_BACKTRACK
            if( theLibCodeRelease.norm(theRobotPose.translation.x() - -4200.f,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > theFieldDimensions.yPosLeftGoal
                    )
                goto gotoGoaliePosition;
#endif
            if (!theLibCodeRelease.isGoalieInAngle ) {
                goto turnToOpponentGoal;
            }



        }
        action
        {

            if(theLibCodeRelease.timeSinceBallWasSeen < 2000 )
                lookAtBall();
            else
                lookLeftAndRight();


            if( !theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), -4200.f, SPQR::GOALIE_POSE_X_TOLLERANCE)
                    || !theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), theLibCodeRelease.glob2Rel(-4200.f, SPQR::GOALIE_BASE_POSITION_Y));
        }
    }

    state(turnToOpponentGoal) // Actually turn to the "right" position according to the ball position.
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "turntoopponentgoal" << std::endl;
#endif
            if( theLibCodeRelease.isGoalieInAngle )
                goto mainLoop;
        }
        action
        {

            lookLeftAndRight();
            if( theRobotPose.rotation >= 0.17f)
                WalkAtAbsoluteSpeed(Pose2f(-.6f, 0.f, 0.f));
            else if ( theRobotPose.rotation < -0.17f)
                WalkAtAbsoluteSpeed(Pose2f(.6f, 0.f, 0.f));
        }
    }

    state(mainLoop)
    {
        transition
        {
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;
#ifdef SPQR_GOALIE_STAND
          if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){
#else
          if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){ //& theBallModel.estimate.position.x() < 1500){
#endif
//              std::cout<<"velocita   "<<velocity.norm()<<std::endl;
//              std::cout<<"posizione   "<<position.norm()<<std::endl;
//              std::cout<<"soglia       "<<velocity.norm()/position.norm()<<std::endl;
              double teta = atan(velocity.y()/velocity.x());
                //std::cout<< "teta   "<<teta<<std::endl;
                float l = position.x()* (float) tan(teta);
                float lato = position.y()-l;
                //std::cout<< "lato  "<< lato <<std::endl;

                if(lato<=200 && lato >= -200)
                    goto stopBall;
                else if(lato>200 && lato < 1500)
                    goto goalieDiveLeft;
                else if(lato<-200 && lato>-1500)
                    goto goalieDiveRight;

            }


#ifndef SPQR_GOALIE_STAND
            if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                    && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                    //&& theLibCodeRelease.isBallInArea      //   -4500 < x < -3900  &&  -1100 < y <1100
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)
                //&& theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )

            {
                goto goalieKickAway;
            }

            if(theLibCodeRelease.timeSinceBallWasSeen < 800){

                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - -4200.f,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 50 )
            {
                goto backTrackInPose;
            }

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - -4200.f,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 500 )
            {
                goto gotoGoaliePosition;
            }
            if (!theLibCodeRelease.isGoalieInAngle) {
                goto turnToOpponentGoal;
            }

#endif



        }
        action
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
                Stand();
                lookAtBall();
            }
#ifndef SPQR_GOALIE_STAND
            else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
                Stand();
                lookAtGlobalBall();
            }
#endif
            else{
                Stand();
                lookLeftAndRight();
            }
        }
    }

    state(goalieOutOfPoles)
        {
            transition
            {
                if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                        && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                        && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                        //&& theLibCodeRelease.isBallInArea      //   -4500 < x < -3900  &&  -1100 < y <1100
                        && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                        && theGameInfo.state == STATE_PLAYING
                        && theBallModel.estimate.position.x() != 0.0
                        && theBallModel.estimate.position.y() != 0.0)
                    //&& theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )

                {
                    goto goalieKickAway;
                }

                if(std::abs(theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x()) < 100 &&
                   std::abs(theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) < 100 &&
                   std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                    goto mainLoop2;

                if(theLibCodeRelease.timeSinceBallWasSeen > 1500 || theBallModel.estimate.position.norm() > 2550)
                    goto gotoGoaliePosition;

//                if(std::abs((theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x()) < 50) && std::abs(theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) < 50)
//                    goto turnToBall;


            }
            action
            {
                    lookAtBall();
                    WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),
                                                                  (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.x(),
                                                                  (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.y()));

    //                if(theLibCodeRelease.isGoalieInArea)
    //                    WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
    //                else if (std::abs(theBallModel.estimate.position.angle()) > 5_deg)
    //                    WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    //                else
    //                    Stand();



    //            else if(theLibCodeRelease.timeSinceBallWasSeen < 6500){
    //                lookLeftAndRight();
    //                Stand();
    //            }
    //            else
    //                lookLeftAndRight(); //lookAtGlobalBall
    //                Stand();
            }


        }

        state(turnToBall)
        {
          transition
          {
              if( theLibCodeRelease.timeSinceBallWasSeen < 800
                      && theBallModel.estimate.velocity.norm() == 0 // valutare di fare 100
                      && theGameInfo.state == STATE_PLAYING
                      && theLibCodeRelease.isBallInKickAwayRange
                      && theLibCodeRelease.isBallInArea
                      && theBallModel.estimate.position.x() != 0.0
                      && theBallModel.estimate.position.y() != 0.0)
                  // && theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3200 )
              {
                  goto goalieKickAway;
              }

              if(theLibCodeRelease.timeSinceBallWasSeen > 1500 || theBallModel.estimate.position.norm() > 2500)
                  goto gotoGoaliePosition;
             if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                 goto mainLoop2;

          }
          action
          {
            lookAtBall();
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
          }
        }

        state(mainLoop2)
        {
          transition
          {

              Vector2f velocity = theBallModel.estimate.velocity;
              Vector2f position = theBallModel.estimate.position;
  #ifdef SPQR_GOALIE_STAND
            if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){
  #else
            if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){ //& theBallModel.estimate.position.x() < 1500){
  #endif
//                std::cout<<"velocita   "<<velocity.norm()<<std::endl;
//                std::cout<<"posizione   "<<position.norm()<<std::endl;
//                std::cout<<"soglia       "<<velocity.norm()/position.norm()<<std::endl;

                double teta = atan(velocity.y()/velocity.x());
                  //std::cout<< "teta   "<<teta<<std::endl;
                  float l = position.x()* (float) tan(teta);
                  float lato = position.y()-l;
                  //std::cout<< "lato  "<< lato <<std::endl;

                  if(lato<=200.f && lato >= -200.f)
                      goto stopBall;
                  else if(lato>200.f && lato < 1500.f)
                      goto goalieDiveLeft;
                  else if(lato<-200.f && lato>-1500.f)
                      goto goalieDiveRight;

              }




            float myDistanceToBall = theBallModel.estimate.position.norm();
            bool iAmMostNearPlayer = true;
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theTeamBallModel.position).norm() < (myDistanceToBall+400.f)){
                    iAmMostNearPlayer = false;
                }
            }



              if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                      && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                      //&& theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                      && theBallModel.estimate.position.norm() < 800
                      && iAmMostNearPlayer
                      //&& theLibCodeRelease.isBallInArea      //   -4500 < x < -3900  &&  -1100 < y <1100
                      //&& theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                      && theGameInfo.state == STATE_PLAYING
                      && theBallModel.estimate.position.x() != 0.0
                      && theBallModel.estimate.position.y() != 0.0)
                  //&& theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()).translation.x() < -3900 )

              {
                  goto goalieKickAway;
              }

              if( theLibCodeRelease.norm(theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x(),
                                      theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) > 150 )
              {
                  goto goalieOutOfPoles;
              }

              if(theLibCodeRelease.timeSinceBallWasSeen > 1800 || theBallModel.estimate.position.norm() > 2550){
//                  std::cout<<"tempo vista palla   "<< theLibCodeRelease.timeSinceBallWasSeen <<std::endl;
//                  std::cout<<"norma   "<< theBallModel.estimate.position.norm() <<std::endl;

                  goto gotoGoaliePosition;

              }


              if(std::abs(theBallModel.estimate.position.angle()) > 5_deg)
              {
                  goto turnToBall;
              }



          }
          action
          {
                  lookAtBall();
                  Stand();
          }
        }

    state(goalieKickAway)
    {
        transition
        {
            //theSPLStandardBehaviorStatus.intention = DROPIN_INTENTION_KEEPER;
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "goaliekickaway" << std::endl;
            std::cerr << "ballsinceseen : " << theLibCodeRelease.timeSinceBallWasSeen << std::endl;
            std::cerr << "state time : " << state_time << std::endl;
#endif

//            if(theLibCodeRelease.timeSinceBallWasSeen < 800){

//                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

//                if(globalCoordBall.translation.x() > -3700 && globalCoordBall.translation.x() < -2500 && velocity.x()<100)
//                    goto goalieOutOfPoles;
//            }

            float myDistanceToBall = theBallModel.estimate.position.norm();
            bool iAmMostNearPlayer = true;
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theTeamBallModel.position).norm() < (myDistanceToBall+400.f)){
                    iAmMostNearPlayer = false;

                }
            }


            if((((int)theGameInfo.setPlay != GOALFREEKICK && !theLibCodeRelease.isGoalieInKickAwayRange && (theBallModel.estimate.position.norm() > 175 && !iAmMostNearPlayer)) ||
                (action_done) ||
                (!theLibCodeRelease.isBallInKickAwayRange && !iAmMostNearPlayer)) ||
               (theLibCodeRelease.timeSinceBallWasSeen > 1000)){
//                std::cerr << "è in area  " << theLibCodeRelease.isGoalieInKickAwayRange << std::endl;
//                std::cerr << "norma della palla  "<<theBallModel.estimate.position.norm() << std::endl;
//                std::cerr << "azione fatta " << action_done << std::endl;
//                std::cerr << "palla è in range kick  " << theLibCodeRelease.isBallInKickAwayRange << std::endl;  //norma < 900
//                std::cerr << "vista da  " << theLibCodeRelease.timeSinceBallWasSeen << std::endl;

                goto gotoGoaliePosition;
            }
            /*if( theLibCodeRelease.timeSinceBallWasSeen() > 3000 && !theLibCodeRelease.isBallInKickAwayRange() )
                goto mainLoop;*/
        }
        action
        {

            //  if(theLibCodeRelease.timeSinceBallWasSeen() > 2000)
            //    lookLeftAndRight();
            //else
            //~ lookAtGlobalBall();
            lookAtBall();
            //std::cerr << "seen ball at : " << theBallModel.estimate.position.x() << ", " << theBallModel.estimate.position.y() << std::endl;
            Pose2f globBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
            if(globBall.translation.y() < - 600)
                //GoAndKick(Pose2f(2000, -3000));
                FastApproacher(Vector2f(5000.f,0.f),true);
            else if(globBall.translation.y() >  600)
                //GoAndKick(Pose2f(2000, 3000));
                FastApproacher(Vector2f(5000.f,0.f),true);
            else
                //GoAndKick(thePossiblePlan.betterTarget);
                FastApproacher(Vector2f(5000.f,0.f),true);


        }
    }

    state(goalieDiveLeft)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto mainLoop;
#endif
#ifndef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            lookAtBall();
            goalieFastDiveLeft();
        }
    }

    state(goalieDiveRight)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto mainLoop;
#endif
#ifndef SPQR_GOALIE_STAND
            if (state_time > SPQR::GOALIE_DIVE_TIME)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            lookAtBall();
            goalieFastDiveRight();
        }
    }

    state(stopBall)
    {
        transition
        {
#ifdef SPQR_GOALIE_STAND
            //todo MARCO goalie dive time too small,
            //if (state_time > SPQR::GOALIE_DIVE_TIME)
            if (state_time > 5000)
                goto mainLoop;
#endif
#ifndef SPQR_GOALIE_STAND
            if (state_time > 3000)
                goto gotoGoaliePosition;
#endif
        }
        action
        {
            lookAtBall();
            StopBall();
        }
    }

}
