#define AU_STATIC_THRESHOLD 100.f
#define CORNER_STATE 3

option(DefenderAU)
{

    initial_state(start)
    {
        transition
        {
            goto walkToPose;
        }
        action
        {
            WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                         Pose2f(theLibCodeRelease.defenderPosition.x(),
                                theLibCodeRelease.defenderPosition.y()));
            LookForward();
        }
    }

    state(coverUp)
    {
        transition
        {
            float defenderBallY = theLibCodeRelease.defenderDynamicY();
            Vector2f defenderCoverPos = Vector2f(theLibCodeRelease.defenderPosition.x(), defenderBallY);
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if ball is far, come back to deafult position
            if ( theTeamBallModel.position.x() >= 400.f || (int)theGameInfo.setPlay == CORNER_STATE )
                    goto walkToPose;
            // approach the ball during the walkPlanner or if the ball is behind the defender
            // if (   (theBallModel.estimate.position.norm() <= 9*AU_STATIC_THRESHOLD || theTeamBallModel.position.x() < theLibCodeRelease.defenderPosition.x())
            //         && theLibCodeRelease.defenderNearestBall() )
            //         goto engageBall;
            if((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD &&
                std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y())))  < 20.f    )
                    goto wait;
            else if((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD)
                    goto turnToBall;
        }
        action
        {
            float defenderBallY = theLibCodeRelease.defenderDynamicY();
            float DefenderBallAngle = theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            Pose2f relTarget = theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(), defenderBallY);

            // to walk always whit angleToGoal
            // if( std::abs(defenderBallY - theLibCodeRelease.defenderPosition.y() ) < AU_STATIC_THRESHOLD ){
            //     WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
            //                 Pose2f(DefenderBallAngle ,
            //                     theLibCodeRelease.defenderPosition.x(),
            //                     defenderBallY));
            // }
            // else {  WalkToTarget(Pose2f(0.5f, 1.f, 1.f),
            //                 Pose2f(DefenderBallAngle ,
            //                     relTarget.translation.x(),
            //                     relTarget.translation.y() ) );
            WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                                    Pose2f(DefenderBallAngle ,
                                    theLibCodeRelease.defenderPosition.x(),
                                    defenderBallY));
            //head
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
                lookAtBall();
            } else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
                lookAtGlobalBall();
            } else {
                LookForward();
            }

        }
    }
    state(turnToBall)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if i'm in my angle's range to the TeamBall
            if( std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 15.f)
                    goto wait;
        }
        action
        {
            float angle = theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()));
            if(angle   > 0.f )  WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
            else WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));

        }
    }

    state(walkToPose)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if the ball is visible and teamBall is valid go to cover-up a dynamic position
            if( (theLibCodeRelease.timeSinceBallWasSeen < 2000 || theTeamBallModel.isValid ) &&
                 theTeamBallModel.position.x()< 400.f && (int)theGameInfo.setPlay != CORNER_STATE ) goto coverUp;
            // if i'm in my position's range && if i'm in my angle's range to lookforward
            if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() < AU_STATIC_THRESHOLD &&
                std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToGoal))  < 10.f    )
                    goto wait;
            // if i'm in my position's range && corner state
            if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() < AU_STATIC_THRESHOLD && (int)theGameInfo.setPlay == CORNER_STATE )
                    goto turnToBall;
            // if i'm in my position's range
            else if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() < AU_STATIC_THRESHOLD)
                    goto turnToPose;
        }
        action
        {
            WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                        Pose2f(theLibCodeRelease.angleToGoal ,
                            theLibCodeRelease.defenderPosition.x(),
                            theLibCodeRelease.defenderPosition.y()));
            //head
            LookForward();
        }
    }

    state(turnToPose)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // if i'm in my angle's range
            if( std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToGoal))  < 10.f   )
                    goto wait;
        }
        action
        {
            if((theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToGoal))   > 0.f )  WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
            else WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));

        }
    }

    state(wait)
    {
        transition
        {
            // moveAway if some robot without pathPlanner is coming toward you
            if( (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() < 800.f) {
                goto moveAway;
            }
            // check to engage the ball
            Vector2f ballVelocity = theBallModel.estimate.velocity;
            Vector2f ballPosition = theBallModel.estimate.position;
            // StopBall if the ball comes towards me
            if(theGameInfo.state == STATE_PLAYING  && ballVelocity.x()<0
                && ballVelocity.norm()/ballPosition.norm()>2.f && ballVelocity.norm() > 20000.f){
                double teta = atan(ballVelocity.y()/ballVelocity.x());
                float l = (float)(ballPosition.x()*tan(teta));
                float lato = ballPosition.y()-l;
                if(lato<=200 && lato >= -200)
                    goto stopBall;
            }
            // engage the ball just if i'm the nearest and if the ball is visible && doesn't move && is in range of 900.f
            // if (theLibCodeRelease.defenderNearestBall() && theLibCodeRelease.timeSinceBallWasSeen < 2000 &&
            //     ballVelocity.norm() < 200.f &&  (ballPosition.norm() <= 9*AU_STATIC_THRESHOLD || theTeamBallModel.position.x() < theLibCodeRelease.defenderPosition.x())
            //     && (int)theGameInfo.setPlay != CORNER_STATE  && theLibCodeRelease.defenderNearestBall() ) goto engageBall;
            // track the ball to cover-up
            float distance = theLibCodeRelease.defenderDynamicDistance();
            if( distance >  800.f && theTeamBallModel.isValid && theTeamBallModel.position.x() < 400.f && (int)theGameInfo.setPlay != CORNER_STATE ) {
                goto coverUp;
            }
            // if the teamBall is not valid come back to default position
            if( (!theTeamBallModel.isValid || theTeamBallModel.position.x() >= 400.f)
                && (theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() > 2*AU_STATIC_THRESHOLD ) {
                goto walkToPose;
            }


        }
        action
        {
            //body
            Stand();
            //head
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
                lookAtBall();
            } else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
                lookAtGlobalBall();
            } else {
                (theBallModel.lastPerception.y() > 0) ?
                            Esorcista(1) : Esorcista(-1);
            }
        }
    }

    state(engageBall)
    {
        transition
        {
            // if the ball is far or not visible, come back to default position || or the ball is moved away by others ||
            // or the ball is kicked come back to defender positio
          if( (!theLibCodeRelease.defenderNearestBall() && theBallModel.estimate.position.norm() > 7*AU_STATIC_THRESHOLD) || ( theBallModel.estimate.position.norm() > 7*AU_STATIC_THRESHOLD && theTeamBallModel.position.x() > theLibCodeRelease.defenderPosition.x()) ||
              ((state_time > 5 && action_done) && theBallModel.estimate.position.norm() > 500.f && theTeamBallModel.position.x() > theLibCodeRelease.defenderPosition.x()))
                    goto walkToPose;
            // if the ball is covered by opponent, dribble
            if(theLibCodeRelease.distance(theLibCodeRelease.activeOpponent(),theTeamBallModel.position) < 1000.f &&
                std::abs(theLibCodeRelease.activeOpponent().translation.y() - theRobotPose.translation.y() ) <= 200.f  ) goto dribble ;
        }
        action
        {
            //head
            lookAtBall();
            if(theTeamBallModel.position.x() > theRobotPose.translation.x()){
                // ChinaApproacher(true, Pose2f(4500,0));
                // included the function to pass
                ChinaApproacher(true, theLibCodeRelease.poseToPass() );
            }
            else {
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(theTeamBallModel.position.x()-100.f,theTeamBallModel.position.y()) );
            }
        }
    }

    state(stopBall)
    {
        transition
        {
            if (state_time > 5000)  goto wait;
        }
        action
        {
            lookAtBall();
            StopBall();
        }
    }

    state(dribble)
    {
        transition
        {
            // if non more obstacle kick away the ball
            if( (std::abs(theLibCodeRelease.activeOpponent().translation.y() - theRobotPose.translation.y() ) >= 200.f ||
                theTeamBallModel.position.x() < theLibCodeRelease.defenderPosition.x() ) &&  theLibCodeRelease.defenderNearestBall() )  goto engageBall;
        }
        action
        {
            Dribble();
        }
    }

    state(moveAway)
    {
        transition
        {
            float defenderBallY = theLibCodeRelease.defenderDynamicY();
            Vector2f defenderCoverPos = Vector2f(theLibCodeRelease.defenderPosition.x(), defenderBallY);
            // for(const auto& mate : theTeamData.teammates){
            //     // if( ((theRobotPose.translation - defenderCoverPos).norm() < AU_STATIC_THRESHOLD ||
            //     //     (theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() < AU_STATIC_THRESHOLD )
            //     //     && (theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() >= AU_STATIC_THRESHOLD  )  goto walkToPose;
            //     if((theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() >= AU_STATIC_THRESHOLD  )  goto walkToPose;
            // }
            if((theLibCodeRelease.nearestTemmate().translation- theRobotPose.translation).norm() >= 800.f  )  goto walkToPose;
        }
        action
        {
            Vector2f defenderCoverPos = Vector2f(theLibCodeRelease.defenderPosition.x(), theLibCodeRelease.defenderDynamicY());
            float DefenderBallAngle = theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            Pose2f relTarget1 = theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(), theRobotPose.translation.y()+300.f);
            Pose2f relTarget2 = theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(), theRobotPose.translation.y()-300.f);
            
            if( theLibCodeRelease.nearestTemmate().translation.y() <  theRobotPose.translation.y()) 
                // WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
                //                         Pose2f(DefenderBallAngle ,
                //                         theLibCodeRelease.defenderPosition.x(),
                //                         theRobotPose.translation.y()+300.f));
                 WalkToTarget(Pose2f(0.5f, 1.f, 1.f),
                            Pose2f(DefenderBallAngle ,
                                relTarget1.translation.x(),
                                relTarget1.translation.y() ) );
            else //WalkToTargetPathPlanner(Pose2f(1.f, 1.f, 1.f),
            //                             Pose2f(DefenderBallAngle ,
            //                             theLibCodeRelease.defenderPosition.x(),
            //                             theRobotPose.translation.y()-300.f));
                 WalkToTarget(Pose2f(0.5f, 1.f, 1.f),
                            Pose2f(DefenderBallAngle ,
                                relTarget2.translation.x(),
                                relTarget2.translation.y() ) );
        }
    }
}
