/** behavior for the ready state */

bool leftEntering=false;

//TODO COMMENT AFTER CC DEMO OR IF YOU DON'T WANT CHEERS
//#define CC_CHEERS

#ifdef CC_CHEERS
    uint pastScore = 0;
#endif


option(ReadyState)
{
#ifndef PENALTY_STRIKER_GOALIE
    initial_state(start)
    {
        transition
        {
            if(state_time > 300)
                goto wait;
#ifdef CC_CHEERS
            if(static_cast<int>(theOwnTeamInfo.score) > (int)pastScore)
                goto cheerNow;
#endif
        }
        action
        {

            if(theRobotPose.translation.y() > 0.0) leftEntering = true;

            Stand();
            LookForward();
        }
    }

#ifdef CC_CHEERS
    state(cheerNow){
        transition
        {
            if(state_time > 3000 || (state_time > 10 && action_done))
                goto start;
        }
        action
        {
            pastScore = static_cast<int>(theOwnTeamInfo.score);
            Cheers("cheer");
        }
    }
#endif

    state(wait)
    {
        transition
        {
            if( theRobotInfo.number == 1 ) goto gotoPosition;
            else if( theRobotInfo.number == 5 ) goto gotoPosition;
            else if( theRobotInfo.number == 2 && state_time > 1000 ) goto gotoPosition;
            else if( state_time > 5000 ) goto gotoPosition;
            else goto gotoPosition;    // maybe to overcome a state_time bug

        }
        action
        {
            Stand();
            lookLeftAndRight();
        }
    }

    target_state(gotoPosition)
    {
        transition
        {

        }
        action
        {
//            if( theRobotInfo.number == 1 ) theHeadControlMode = HeadControl::lookLeftAndRightUpAndDown;
            lookLeftAndRight();
            // // if i have all the robots use standard number assignment
            // if(theTeamData.teammates.size() == 4 ){
                if(theRobotInfo.number == 1)
                    Goalie();
                else
                {   if(theRole.role == Role::RoleType::undefined){
                        if( theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber )
                        {
                            Pose2f readyPose(theLibCodeRelease.getReadyPose(false, (Role::RoleType)(theRobotInfo.number) ));
                            GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
                        }
                        else
                        {
                            Pose2f readyPose(theLibCodeRelease.getReadyPose(true, (Role::RoleType)(theRobotInfo.number) ));
                            GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
                        }
                    }
                    else
                    {
                            if( theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber )
                            {
                                Pose2f readyPose(theLibCodeRelease.getReadyPose(false, (Role::RoleType)(theRobotInfo.number) ));
                                GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
                            }
                            else
                            {
                                Pose2f readyPose(theLibCodeRelease.getReadyPose(true, (Role::RoleType)(theRobotInfo.number) ));
                                GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
                            }
                    }

                }
            // }
            // else{
            //     // ordering teammates
            //     std::vector<int> allActiveNumbers;
            //     std::vector<Teammate> teammates = theTeamData.teammates;
            //     // std::cout<<"giocatori "<<teammates.size() <<std::endl;
            //      for(int j = 0; j < teammates.size(); j++ ){
            //             allActiveNumbers.push_back(teammates.at(j).number);
            //         }
            //     allActiveNumbers.push_back(theRobotInfo.number);
            //     // std::cout<<"numeri attivi "<<teammates.size() <<std::endl;
            //     int swapper;
            //     for(int i =0; i < allActiveNumbers.size(); i++ ){
            //         for(int k = 0; k < allActiveNumbers.size(); k++ ){
            //             if(allActiveNumbers.at(i) > allActiveNumbers.at(k) ){
            //                 swapper = allActiveNumbers.at(k);
            //                 allActiveNumbers.at(k) = allActiveNumbers.at(i);
            //                 allActiveNumbers.at(i) = swapper;
            //             }
            //         }
            //     }
            //     // assign role with incremental role importance
            //     for(int i = 0; i < allActiveNumbers.size(); i++ ){
            //         if(allActiveNumbers.at(i)== theRobotInfo.number ){    
            //             if(theRole.role == Role::RoleType::undefined){
            //                 if( theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber )
            //                 {
            //                     Pose2f readyPose(theLibCodeRelease.getReadyPose(false, (Role::RoleType)(i+1) ));
            //                     GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
            //                 }
            //                 else
            //                 {
            //                     Pose2f readyPose(theLibCodeRelease.getReadyPose(true, (Role::RoleType)(i+1) ));
            //                     GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
            //                 }
            //             }
            //             else
            //             {
            //                     if( theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber )
            //                     {
            //                         Pose2f readyPose(theLibCodeRelease.getReadyPose(false, (Role::RoleType)(i+1) ));
            //                         GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
            //                     }
            //                     else
            //                     {
            //                         Pose2f readyPose(theLibCodeRelease.getReadyPose(true, (Role::RoleType)(i+1) ));
            //                         GetIntoReadyPosition(Pose2f(.8f, .8f, .8f), readyPose.translation );
            //                     }
            //             }
            //         }
            //     }

            // }
        }
    }
#else
     initial_state(start)
    {
        transition
        {

        }
        action
        {



            //Stand();
            LookForward();
        }
    }
#endif

}
