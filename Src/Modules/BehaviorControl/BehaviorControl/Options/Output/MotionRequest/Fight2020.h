
option(Fight2020,(const Pose2f&) activeOpponent)
{
    Pose2f globball = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    
    initial_state(start)
    {

            transition
            {   
                if(activeOpponent.translation.x() < theRobotPose.translation.x() && globball.translation.x() < theRobotPose.translation.x()){
                    if(globball.translation.y() < theRobotPose.translation.y()){
                        goto reachBallSideLeft;
                    }else{
                        goto reachBallSideRight;
                    }
                }
                else{
                    goto reachBallBack;
                }
                
                
            }
            action
            {
                Stand();

            }
    }

    state(reachBallBack){
        transition{
            if(theBallModel.estimate.position.norm() < 400.f){
                if( theLibCodeRelease.between(theRobotPose.translation.x(), globball.translation.x() - 180 , globball.translation.x() - 30) ){
                    goto cover;
                }
                
            }
            if(activeOpponent.translation.x() < theRobotPose.translation.x() && globball.translation.x() < theRobotPose.translation.x()){
                goto start;
            }
        }action{
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(0, globball.translation.x()-150.f,globball.translation.y()) );
            lookAtBall();
        }
    }

    state(reachBallSideLeft){
        transition{
            if(std::abs(theLibCodeRelease.angleToTarget(globball.translation.x(),globball.translation.y() ) )< Angle::fromDegrees(15)){
                if(theBallModel.estimate.position.x() < 180){
                    goto InWalkKick;
                }
                
            }
        }action{
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(-1.57, globball.translation.x(),globball.translation.y() + 150.f) );
            lookAtBall();
        }
    }

    state(reachBallSideRight){
            transition{
                if(std::abs(theLibCodeRelease.angleToTarget(globball.translation.x(),globball.translation.y() ) )< Angle::fromDegrees(15)){
                    if(theBallModel.estimate.position.x() < 180){
                        goto InWalkKick;
                    }
                
                }
            }action{
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(1.57, globball.translation.x(),globball.translation.y() - 150.f) );
                lookAtBall();
            }
    }

    state(InWalkKick){
       transition{
           if(state_time > 2000 || action_done || theBallModel.estimate.position.norm() > 400.f || theBallModel.estimate.position.x() < -10){
               goto start;
           }
       }action{
            lookAtBall();
            if(theBallModel.estimate.position.y() <= 0){
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

            }else{
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
    
            }
           
           
        }
   }

    state(cover){
        transition{
            if(theBallModel.estimate.position.norm() > 450.f){
                goto start;
            }
            if(activeOpponent.translation.x() < theRobotPose.translation.x() && globball.translation.x() < theRobotPose.translation.x()){
                goto start;
            }
            if( theLibCodeRelease.between( theBallModel.estimate.position.x(),0, 180.f)){
                if(!theLibCodeRelease.between( theLibCodeRelease.glob2Rel(activeOpponent.translation.x(), activeOpponent.translation.y() ).translation.y(), 
                 -150, 150)){
                     goto InWalkKick;
                 }
            }
        }action{
            WalkAtAbsoluteSpeed(Pose2f(theLibCodeRelease.angleToTarget(4500.f,theRobotPose.translation.y()), theBallModel.estimate.position.x() - 30.f,
             theBallModel.estimate.position.y()));
             lookAtBall();
        }
    }

}
