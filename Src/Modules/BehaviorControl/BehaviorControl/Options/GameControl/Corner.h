option(Corner)
{
    initial_state(corner)
    {
        transition
        {
           if((int)theGameInfo.kickingTeam==Global::getSettings().teamNumber)
                goto offensiveCorner;
           else
                goto defensiveCorner;
        }
    }

    state(offensiveCorner)
    {
        action
        {
            if(theRole.role == Role::RoleType::striker || theRole.role == Role::RoleType::searcher_3 )
            {
                if( theLibCodeRelease.timeSinceBallWasSeen > 2000.f || !theTeamBallModel.isValid ){
                    WalkToTargetPathPlanner( Pose2f(0.8f,0.8f,0.8f), Pose2f( theFieldDimensions.xPosOpponentGroundline-1000.f, theFieldDimensions.yPosRightDropInLine+500.f) );
                }
                else ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline-1000, 0.f) ); 
            }    
            if(theRole.role == Role::RoleType::jolly || theRole.role == Role::RoleType::searcher_4)
            {
                if( theLibCodeRelease.timeSinceBallWasSeen > 2000.f || !theTeamBallModel.isValid ){
                    WalkToTargetPathPlanner( Pose2f(0.8f,0.8f,0.8f), Pose2f( theFieldDimensions.xPosOpponentGroundline-1000.f, theFieldDimensions.yPosLeftDropInLine-500.f) );
                }
                else WalkToTargetPathPlanner(Pose2f(0.8f,0.8f,0.8f), Pose2f(theBallModel.estimate.rotation, theFieldDimensions.xPosOpponentGroundline-1000.f, 0.f)); 
            }    
            else Play();
        }
        
    }

    state(defensiveCorner)
    {
        action
        {
            float angleCornerLeft = theLibCodeRelease.angleToTarget(-4500.f,3000.f);
            float angleCornerRight = theLibCodeRelease.angleToTarget(-4500.f,-3000.f);
            if(theRole.role == Role::RoleType::striker || theRole.role == Role::RoleType::searcher_1)
                if( theLibCodeRelease.timeSinceBallWasSeen > 2000.f || !theTeamBallModel.isValid ){
                        WalkToTargetPathPlanner( Pose2f(0.8f,0.8f,0.8f), Pose2f( angleCornerLeft,theFieldDimensions.xPosOwnPenaltyArea+1000.f, theFieldDimensions.yPosRightDropInLine+500.f) );
                    }
                else CoverCornerKick(0);           
            else if(theRole.role == Role::RoleType::supporter || theRole.role == Role::RoleType::searcher_2)
                if( theLibCodeRelease.timeSinceBallWasSeen > 2000.f || !theTeamBallModel.isValid ){
                        WalkToTargetPathPlanner( Pose2f(0.8f,0.8f,0.8f), Pose2f( angleCornerRight,theFieldDimensions.xPosOwnPenaltyArea+1000.f, theFieldDimensions.yPosLeftDropInLine-500.f) );
                    }
                else CoverCornerKick(1);
            else
                Play();
            //TODO  set defense position
        }
    }
}