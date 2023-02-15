float setTR;

// THIS TO HANDLE GOING BEHIND STRIKER
// Trigger this if(theRobotPose.translation.x() > strikerPosition.x())
option(GoBehindStriker)
{
    initial_state(get_behind_striker)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 800.f){
                if(theRobotPose.translation.y() < strikerPosition.y()){
                    if(theRobotPose.translation.y() < 700.f - theFieldDimensions.yPosLeftSideline){
                        if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 500.f){
                            setTR = (float)0.16;
                            goto do_a_semicircle_up;
                        }
                        else
                            goto walk_straight_back;
                    }
                    else{
                        setTR = (float)0.3;
                        goto do_a_semicircle_down;
                    }
                }
                else{
                    if(theRobotPose.translation.y() > theFieldDimensions.yPosLeftSideline - 700.f){
                        if(std::abs(theRobotPose.translation.y() - strikerPosition.y()) < 500.f){
                            setTR = (float)0.16;
                            goto do_a_semicircle_down;
                        }
                        else
                            goto walk_straight_back;
                    }
                    else{
                        setTR = (float)0.3;
                        goto do_a_semicircle_up;
                    }
                }
            }
            else {
                goto walk_straight_back;
            }
        }
        action
        {
            ;
        }
    }

    // These movements need to check for collision with opponent
    state(do_a_semicircle_down)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < strikerPosition.x())
                goto stand;

            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0 && obs.center.x() < 500.f){
                    goto walk_straight_back;
                }
            }

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            LookForward();
            WalkQuarterCircleDownLeft(setTR);
        }
    }

    state(do_a_semicircle_up)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < strikerPosition.x())
                goto stand;

            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0.f && obs.center.x() < 500.f){
                    goto walk_straight_back;
                }
            }

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            LookForward();
            WalkQuarterCircleUpLeft(setTR);
        }
    }


    state(walk_straight_back)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() < strikerPosition.x())
                goto stand;

            if(state_time > 3000.f)
                goto get_behind_striker;
        }
        action
        {
            LookForward();

            bool ObsAvoid = false;
            for(const auto& obs : theObstacleModel.obstacles){
                Vector2f globalObsPos = theLibCodeRelease.rel2Glob(obs.center.x(),obs.center.y()).translation;
                if(globalObsPos.x() < theRobotPose.translation.x() && std::abs(globalObsPos.x()-theRobotPose.translation.x()) < 500.f ) {
                    if(std::abs(globalObsPos.y()-theRobotPose.translation.y()) < 500.f){
                        if(globalObsPos.y() < theRobotPose.translation.y())
                            WalkToTarget(Pose2f(1,1,1),theLibCodeRelease.glob2Rel(theRobotPose.translation.x(),theRobotPose.translation.y()+500.f));
                        else
                            WalkToTarget(Pose2f(1,1,1),theLibCodeRelease.glob2Rel(theRobotPose.translation.x(),theRobotPose.translation.y()-500.f));
                        ObsAvoid = true;
                        break;
                    }
                }
            }

            if(!ObsAvoid){
                Vector2f targetPosition(theRobotPose.translation.x()-500.f,theRobotPose.translation.y());
                Vector2f relTargetPos = theLibCodeRelease.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
                WalkToTarget(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
            }

        }
    }

    state(stand)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            if(theRobotPose.translation.x() > strikerPosition.x())
                goto get_behind_striker;
        }
        action
        {
            lookLeftAndRight();
            Stand();
        }
    }
}