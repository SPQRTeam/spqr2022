option(CoverCornerKick, (const int) player)
{    

    initial_state(start)
    {
        transition
        {
            if(theTeamBallModel.isValid)
            {
                if(theTeamBallModel.position.y() < 1000)
                    goto rightCorner;
                else
                    goto leftCorner;
            }
        }
        action
        {
            lookLeftAndRight();
            Stand();
        }
    }

    state(rightCorner)
    {
        transition
        {
            if( player == 0 && 
                (theRobotPose.translation - Pose2f(theFieldDimensions.xPosOwnPenaltyArea-200.f, theFieldDimensions.yPosRightDropInLine+500.f).translation).norm() < 200.f ) 
                goto target;

            if( player != 0 && 
                (theRobotPose.translation - Pose2f(theFieldDimensions.xPosOwnPenaltyArea+100.f, theFieldDimensions.yPosRightDropInLine+100.f).translation).norm() < 200.f ) 
                goto target;
            
        }
        action
        {
            lookAtGlobalBall();
            if(player == 0)
                WalkToTargetPathPlanner(Pose2f(80.0f, 80.0f, 80.0f), Pose2f(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theFieldDimensions.xPosOwnPenaltyArea-200.f, theFieldDimensions.yPosRightDropInLine+500.f)); 
            else
                WalkToTargetPathPlanner(Pose2f(80.0f, 80.0f, 80.0f), Pose2f(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theFieldDimensions.xPosOwnPenaltyArea+100.f, theFieldDimensions.yPosRightDropInLine+100.f)); 
        }
    }

    state(leftCorner)
    {
        transition
        {
            if( player == 0 && 
                (theRobotPose.translation - Pose2f(theFieldDimensions.xPosOwnPenaltyArea-200.f, theFieldDimensions.yPosLeftDropInLine-500.f).translation).norm() < 200.f ) 
                goto target;

            if( player != 0 && 
                (theRobotPose.translation - Pose2f(theFieldDimensions.xPosOwnPenaltyArea+100.f, theFieldDimensions.yPosLeftDropInLine-100.f).translation).norm() < 200.f ) 
                goto target;
            
        }
        action
        {
            lookAtGlobalBall();
           if(player == 0)
                WalkToTargetPathPlanner(Pose2f(80.0f, 80.0f, 80.0f), Pose2f(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theFieldDimensions.xPosOwnPenaltyArea-200.f, theFieldDimensions.yPosLeftDropInLine-500.f)); 
            else
                WalkToTargetPathPlanner(Pose2f(80.0f, 80.0f, 80.0f), Pose2f(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()), theFieldDimensions.xPosOwnPenaltyArea+100.f, theFieldDimensions.yPosLeftDropInLine-100.f)); 
        }
    }
    state(target)
    {
        action
        {
            //std::cout<< theLibCodeRelease.norm(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()) <<std::endl;
            Stand();
        }
    }


}