option(Dribble)
{   
    Pose2f globbal = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y() );
  
    initial_state(start)
    {
        
        Pose2f opponentPose = theLibCodeRelease.activeOpponent();
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
            if(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y() > 0 &&
             theBallModel.estimate.position.norm() < 300.f)
                goto preStepRight;
            else
                goto preStepLeft;

        }
        action
        {   
            lookAtBall();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), globbal-Pose2f(300.f,0.f));
        }
    }

    state(preStepRight)
    {
        Pose2f opponentPose = theLibCodeRelease.activeOpponent();
        transition
        {

            if(theBallModel.estimate.position.norm() > 500 || globbal.translation.x() > theRobotPose.translation.x() -10){
                goto start;
            }
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f))
            {
                if(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y() < 200)
                {
                    //std::cout<<"dribbleRight  "<<theRobotPose.rotation<<std::endl;
                    goto dribbleRight;
                }
                else
                {
                    //std::cout<<"forwardInWalkRight"<<std::endl;
                    goto forwardInWalkRight;
                }
            }
        }
        action
        {
            lookAtBall();
            WalkToTarget(Pose2f(1.f, 1.f, 1.f), Pose2f(0.f,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()+30.f));
        }
    }

    state(preStepLeft)
    {        
        Pose2f opponentPose = theLibCodeRelease.activeOpponent();
        transition
        {
            if(theBallModel.estimate.position.norm() > 500 || globbal.translation.x() < theRobotPose.translation.x() +10){
                goto start;
            }
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f))
            {
            
                if(theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y() > -200)
                {
                    //std::cout<<"dribbleLeft   "<<theLibCodeRelease.glob2Rel(opponentPose.translation.x(), opponentPose.translation.y()).translation.y()<<std::endl;
                    goto dribbleLeft;
                }
                else
                {
                    //std::cout<<"forwardInWalkLeft"<<std::endl;
                    goto forwardInWalkLeft;
                }
            }
        }
        action
        {
            lookAtBall();
            WalkToTarget(Pose2f(1.f, 1.f, 1.f), Pose2f(0.f,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()-30.f));
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
    state(forwardInWalkRight)
    {
        transition
        {
            //std::cout<<"forward right"<<std::endl;
            if(action_done)
                goto target;
        }
        action
        {
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
    }

    state(forwardInWalkLeft)
    {
        transition
        {
            //std::cout<<"forward left"<<std::endl;
            if(action_done)
                goto target;
        }
        action
        {
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        
        }
    }

    target_state(target)
    {   
        action
        {
            //Stand();
            WalkToTarget(Pose2f(1.f, 1.f, 1.f), Pose2f(theBallModel.estimate.rotation,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()-40.f));
        }
    }
}
