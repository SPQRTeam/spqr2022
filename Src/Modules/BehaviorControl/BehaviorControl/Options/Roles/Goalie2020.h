//#define SPQR_DEBUG_GOALIE

option(Goalie2020)
{
    common_transition
    {
        if((int)theGameInfo.setPlay == GOALFREEKICK) 
            goto goalieKickAway;

        Vector2f velocity = theBallModel.estimate.velocity;
        Vector2f position = theBallModel.estimate.position;
        
        if(theGameInfo.state == STATE_PLAYING  && velocity.x() < 100 && velocity.norm()/position.norm()>1.8)
            goto goalieCore;
    }

    initial_state(start)
    {
        transition
        {
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
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

            if(theLibCodeRelease.timeSinceBallWasSeen < 800){

                Vector2f velocity = theBallModel.estimate.velocity;
                Vector2f position = theBallModel.estimate.position;
                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && 
                  globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), (theFieldDimensions.xPosOwnGroundline + 300) + theLibCodeRelease.goalie_displacement, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                goto turnToOpponentGoal;

            if( theLibCodeRelease.isGoalieInAngle )
                goto backTrackInPose;
        }
        action
        {
            if(std::abs(theLibCodeRelease.angleToTarget( (theFieldDimensions.xPosOwnGroundline + 300) + theLibCodeRelease.goalie_displacement,
                                                     SPQR::GOALIE_BASE_POSITION_Y)) > 10.f/*Angle::fromDegrees(10.f)*/){
                lookLeftAndRight();
                WalkToTarget(Pose2f(50.f, 50.f, 50.f),
                         Pose2f(theLibCodeRelease.angleToTarget( (theFieldDimensions.xPosOwnGroundline + 300) +theLibCodeRelease.goalie_displacement,
                                                             SPQR::GOALIE_BASE_POSITION_Y), 0.f, 0.f));
            }
            else{
                lookLeftAndRight();

                WalkToTarget(Pose2f(50.f,50.f,50.f),
                          theLibCodeRelease.glob2Rel( (theFieldDimensions.xPosOwnGroundline + 300) +theLibCodeRelease.goalie_displacement,
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
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

            if(theLibCodeRelease.timeSinceBallWasSeen < 800){

                Vector2f velocity = theBallModel.estimate.velocity;
                Vector2f position = theBallModel.estimate.position;
                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && 
                  globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack balanced" <<  (theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                                                   theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE)) << std::endl;
#endif
            if( theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
                    theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                goto mainLoop;

#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "backtrack " << theLibCodeRelease.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                                                theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) << std::endl;
#endif

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > theFieldDimensions.yPosLeftGoal)
                goto gotoGoaliePosition;

            if (!theLibCodeRelease.isGoalieInAngle )
                goto turnToOpponentGoal;
        }
        action
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000 )
                lookAtBall();
            else
                lookLeftAndRight();

            if( !theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline + 300, SPQR::GOALIE_POSE_X_TOLLERANCE)
                    || !theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), theLibCodeRelease.glob2Rel( (theFieldDimensions.xPosOwnGroundline + 300), SPQR::GOALIE_BASE_POSITION_Y));
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
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "mainLoop" << std::endl;
#endif
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;
            if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7) //& theBallModel.estimate.position.x() < 1500)
                goto goalieCore;

            if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                    && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

            if(theLibCodeRelease.timeSinceBallWasSeen < 800){

                Pose2f globalCoordBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());

                if(globalCoordBall.translation.x() > theFieldDimensions.xPosOwnPenaltyArea+200 && globalCoordBall.translation.x() < -0.55f*SPQR::FIELD_DIMENSION_X && velocity.x()<100)
                    goto goalieOutOfPoles;
            }

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 50 )
                goto backTrackInPose;

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGroundline + 300,
                                    theRobotPose.translation.y() - SPQR::GOALIE_BASE_POSITION_Y) > 500 )
                goto gotoGoaliePosition;

            if (!theLibCodeRelease.isGoalieInAngle) 
                goto turnToOpponentGoal;
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
        }
    }

    state(goalieOutOfPoles)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "goalieOutPoles" << std::endl;
#endif
            if(     theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() < SPQR::MOVING_BALL_MIN_VELOCITY // norm < 200
                    && theLibCodeRelease.isBallInKickAwayRange    // palla < 900
                    && theLibCodeRelease.isGoalieInKickAwayRange  // -4500 < x < -3700  &&  -1100 < y <1100
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

            if(std::abs(theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x()) < 100 &&
               std::abs(theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) < 100 &&
               std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                goto mainLoop2;

            if(theLibCodeRelease.timeSinceBallWasSeen > 1500 || theBallModel.estimate.position.norm() > 2550)
                goto gotoGoaliePosition;

//          if(std::abs((theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x()) < 50) && std::abs(theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) < 50)
//              goto turnToBall;
        }
        action
        {
            lookAtBall();
            if (between(theRobotPose.rotation, Angle::fromDegrees(-60.f), Angle::fromDegrees(60.f) ))
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(),
                                                          (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.x(),
                                                          (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.y()));
            else 
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(0,
                                                          (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.x(),
                                                          (theLibCodeRelease.glob2Rel(theLibCodeRelease.goaliePosition.x(), theLibCodeRelease.goaliePosition.y())).translation.y()));                
        }
    }

    state(turnToBall)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "turnToBall" << std::endl;
#endif
            if( theLibCodeRelease.timeSinceBallWasSeen < 800
                    && theBallModel.estimate.velocity.norm() == 0 // valutare di fare 100
                    && theGameInfo.state == STATE_PLAYING
                    && theLibCodeRelease.isBallInKickAwayRange
                    && theLibCodeRelease.isBallInArea
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

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
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "mainLoop2" << std::endl;
#endif          
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;
            if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7) //& theBallModel.estimate.position.x() < 1500)
                goto goalieCore;

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
                    && theBallModel.estimate.position.norm() < 800
                    && iAmMostNearPlayer
                    && theGameInfo.state == STATE_PLAYING
                    && theBallModel.estimate.position.x() != 0.0
                    && theBallModel.estimate.position.y() != 0.0)

                goto goalieKickAway;

            if( theLibCodeRelease.norm(theRobotPose.translation.x() - theLibCodeRelease.goaliePosition.x(),
                                    theRobotPose.translation.y() - theLibCodeRelease.goaliePosition.y()) > 150 )
                goto goalieOutOfPoles;

            if(theLibCodeRelease.timeSinceBallWasSeen > 1800 || theBallModel.estimate.position.norm() > 2550)
                goto gotoGoaliePosition;

            if(std::abs(theBallModel.estimate.position.angle()) > 5_deg)
                goto turnToBall;
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
        }
    }

    state(goalieKickAway)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "goaliekickaway" << std::endl;
            std::cerr << "ballsinceseen : " << theLibCodeRelease.timeSinceBallWasSeen << std::endl;
            std::cerr << "state time : " << state_time << std::endl;
#endif
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
               (theLibCodeRelease.timeSinceBallWasSeen > 1000))

                goto gotoGoaliePosition;
        }
        action
        {
            lookAtBall();
            Pose2f globBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
            if(globBall.translation.y() < - 600)
                //GoAndKick(Pose2f(2000, -3000));
                FastApproacher(Vector2f(2000.f,0.f),true);
            else if(globBall.translation.y() >  600)
                //GoAndKick(Pose2f(2000, 3000));
                FastApproacher(Vector2f(2000.f,0.f),true);
            else
                //GoAndKick(thePossiblePlan.betterTarget);
                FastApproacher(Vector2f(2000.f,0.f),true);
        }
    }

    state(goalieCore)
    {
        transition
        {
#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "goInGoalieCore" << std::endl;
#endif          
            if (state_time > 3000)
                goto mainLoop;
        }
        action
        {
            GoalieCore();
        }
    }
}