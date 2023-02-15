option(FreeKick)
{
    initial_state(freeKick)
    {
        transition
        {
           if((int)theGameInfo.kickingTeam==Global::getSettings().teamNumber)
                goto offensiveFreeKick;
           else
                goto defensiveFreeKick;
        }
    }

    state(offensiveFreeKick)
    {
        action
        {
            Play();
        }
    }

    state(defensiveFreeKick)
    {
        action
        {
            if(theRole.role == Role::RoleType::striker || theRole.role == Role::RoleType::searcher_3)
            {
                if(theBallModel.estimate.position.norm() < 800)
                {
                    //TODO improve freekickdistance
                    FreeKickDistance();
                }
                else
                {
                    //TODO Striker defense position
                    Stand();
                }
                
            }
            else
            {
                //TODO defensive position
                Pose2f readyPose(theLibCodeRelease.getReadyPose(true, theRole.role ));
                GetIntoReadyPosition(Pose2f( .6f, 70.f, 20.f), readyPose.translation );
            }
            
        }
    }
}