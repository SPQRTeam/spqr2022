/**
 * Kicks the Ball while moving towards on the y-robot axis. 
 * @param bool side: if true leftInWalkKick, if false right
 */

option(SideKicking, (const bool) side)
{   
    initial_state(start)
    {
        transition
        {   
            if (side)
                goto preStepLeft;
            else if(!side)
                goto preStepRight;       
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
            if(theLibCodeRelease.between(theBallModel.estimate.position.x(), -20.f, 20.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.y(), -210.f, -190.f))
            {
                goto kickRight;
            }
        }
        action
        {
            lookAtBall();
            //if(theLibCodeRelease.between(theLibCodeRelease.angleToGoal, -1.9f, -1.f))
            WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(/* theLibCodeRelease.angleToGoal*/ 0.f, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()+200.f));
        }
    }

    state(preStepLeft)
    {        
        transition
        {
            if(theLibCodeRelease.between(theBallModel.estimate.position.x(), -20.f, 20.f) && 
                theLibCodeRelease.between(theBallModel.estimate.position.y(), 190.f, 210.f))
            {
                goto kickLeft;
            }
        }
        action
        {
            lookAtBall();
            WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(0.f, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()-200.f));
        }
    }

    state(kickLeft)
    {
        transition
        {
            if(action_done)
                goto target;
        }
        action
        {
            //std::cout << "dribbleLeft    "<<theBallModel.estimate.rotation<<std::endl;
            InWalkKick(WalkKickVariant(WalkKicks::sideKick, Legs::right), Pose2f(0.f, 0.f, 0.f));

        }
    }

    state(kickRight)
    {
        transition
        {
            if(action_done)
                goto target;
        }
        action
        {
            //std::cout << "dribbleRight    "<<theBallModel.estimate.position.x()<<std::endl;
            InWalkKick(WalkKickVariant(WalkKicks::sideKick, Legs::left), Pose2f(0.f, 0.f, 0.f));
        }
    }

    target_state(target)
    {   
        action
        {
            Stand();
            //WalkToTarget(Pose2f(1.f, 1.f, 1.f), Pose2f(theBallModel.estimate.rotation,/*theLibCodeRelease.angleToGoal---0.f*/ theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y()));
            
        }
    }
}
