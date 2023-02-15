/**
 * Kicks the Ball with the specified \c kickType.
 * @param bool forward: if true forwardKick, if false sideKick 
 * @param bool side: if true rightInWalkKick, if false left
 */

option(InWalkKickApproacher, (const bool) forward, (const bool) side)
{   
    initial_state(start)
    {
        transition
        {   
            //std::cout << "opponent y:    "<<theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y()<<std::endl;
            /* if(theLibCodeRelease.between(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y(), 0, 200))
                goto preStepRight;
            else if(theLibCodeRelease.between(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y(), -200, 0))
                goto preStepLeft;
            else //if(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y() > 200 ||))
                goto forwardInWalk;
            */
            if(forward)
                goto preStepForward;
            else if (side)
                goto preStepRight;
            else if(!side)
                goto preStepLeft;
           
        }
        action
        {   
            lookAtBall();
            Stand();
        }
    }

    state(preStepRight)
    {
        transition
        {
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f))
            {
                goto dribbleRight;
            }
        }
        action
        {
            lookAtBall();
            //std::cout << "dribbleRight    "<<theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(),theBallModel.estimate.position.y())<< " | " <<theRobotPose.rotation<< std::endl;
            //if(!theLibCodeRelease.between(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()), -0.17f, 0.17f))
              //  WalkToTarget(Pose2f(0.6f, 0.1f, 0.1f), Pose2f(theBallModel.estimate.position.angle(),/*theLibCodeRelease.angleToGoal---0.f*/ 0.f, 0.f));
            //else
                WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(0.f, theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()+40.f));

        }
    }

    state(preStepLeft)
    {        
        transition
        {
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f))
            {
                goto dribbleLeft;
            }
        }
        action
        {
            lookAtBall();
            WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(0.f,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()-40.f));
        }
    }

    state(dribbleLeft)
    {
        transition
        {
            if(action_done)
                goto target;
        }
        action
        {
            //std::cout << "dribbleLeft    "<<theBallModel.estimate.rotation<<std::endl;
            lookAtBall();
            InWalkKick(WalkKickVariant(WalkKicks::left, Legs::right), Pose2f(25_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

        }
    }

    state(dribbleRight)
    {
        transition
        {
            if(action_done)
                goto target;
        }
        action
        {
            //std::cout << "dribbleRight    "<<theBallModel.estimate.position.x()<<std::endl;
            InWalkKick(WalkKickVariant(WalkKicks::right, Legs::left), Pose2f(-25_deg, theBallModel.estimate.position.x(), -theBallModel.estimate.position.y()));
        }
    }
    state(preStepForward)
    {
        transition
        {
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -60.f, 60.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f))
                goto forwardInWalk;
        }
        action
        {
            lookAtBall();
            if(theBallModel.estimate.position.y()>0)
                WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(0.f,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()-50.f));
            else
                WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(0.f,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()+50.f));
            
        }
    }
    state(forwardInWalk)
    {
        transition
        {
            if(action_done)
                goto target;
        }
        action
        {   
            if(theBallModel.estimate.position.y() < 0)
            {   
                //std::cout<<"destra  "<<std::endl;
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
            }
            else
            {
                //std::cout<<"sinistra  "<<std::endl;
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));            
            }    
        }
    }

    target_state(target)
    {   
        action
        {
            //Stand();
            WalkToTarget(Pose2f(0.8f, 0.8f, 0.8f), Pose2f(theBallModel.estimate.rotation,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()));
        }
    }
}
