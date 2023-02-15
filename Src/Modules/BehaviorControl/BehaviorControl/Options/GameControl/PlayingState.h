//#define SINGLE_ROLE
#define getExtraTime() ( (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber) ? 10000.f : 0.f)


option(PlayingState)
{
    initial_state(demo)
    {
        action
        {

#ifndef SINGLE_ROLE
            //  play normal game ||  free kick own team  ||  goal free kick own team  ||  kick in own team
            switch ((int)theGameInfo.setPlay)
            {
                case PLAY:
                    Play();
                    break;
                case GOALFREEKICK:
                    GoalFreeKick();
                    break;
                case FREEKICK:
                    FreeKick();
                    break;
                case CORNER:
                    Corner();
                    break;
                case SIDEKICK:
                    SideKick();
                    break;    
                default:
                    exit (-1);
                    break;
            }

#else       
            Stand();
            //            Goalie();
            //ApproachToTarget(Pose2f((float)theFieldDimensions.xPosOpponentGroundline,0.f));
            //            if(theTeamBallModel.isValid)
            //                lookAtGlobalBall();
            //            else
            //                lookLeftAndRight();

#endif
        }
    }
}

