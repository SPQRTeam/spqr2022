option(SideKick)
{

    initial_state(sideKick)
    {
        transition
        {
            if(theRole.role == Role::RoleType::goalie)
                goto PlayState;
                
            // Our kick
            if((int)theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
                if (theTeamBallModel.position.x()<0)
                {
                    if(theRole.role == Role::RoleType::supporter || theRole.role == Role::RoleType::jolly || theRole.role == Role::RoleType::striker)
                        goto OffensiveSideKick;
                    else
                        goto PlayState;
                }
                else {
                    if(theRole.role == Role::RoleType::jolly || theRole.role == Role::RoleType::striker)
                        goto OffensiveSideKick;
                    else
                        goto PlayState;
                }
            }
            // Their kick
            else {
                if(theTeamBallModel.position.x() < 0){
                    if(theRole.role == Role::RoleType::jolly)
                        goto turnToBall;
                    else
                        goto ownFieldDefSideKick;
                }
                else{
                    if(theRole.role == Role::RoleType::defender)
                        goto PlayState;
                    else
                        goto oppFieldDefSideKick;
                }
            }
        }
    }

    state(PlayState){
        transition
        {
            if(state_time > 5000)
                goto sideKick;
        }
        action
        {
            Play();
        }
    }

    state(OffensiveSideKick)
    {
        transition
        {
            
            if (theTeamBallModel.position.x()<0) {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
                std::tuple<Vector2f,Vector2f> thePositions = theLibCodeRelease.getOwnFieldOffSideKickPos();
                if(theRole.role == Role::RoleType::striker && (theRobotPose.translation-std::get<0>(thePositions)).norm() < 700.f)
                    goto offensiveStand;
                else if((theRobotPose.translation-std::get<0>(thePositions)).norm() < 150.f)
                    goto offensiveTurn;
            }
            else {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::supporter || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
                std::tuple<Vector2f,Vector2f,bool> thePositions = theLibCodeRelease.getOppFieldOffSideKickPos();
                if(theRole.role == Role::RoleType::striker && (theRobotPose.translation-std::get<0>(thePositions)).norm() < 700.f)
                    goto offensiveStand;
                else if((theRobotPose.translation-std::get<0>(thePositions)).norm() < 150.f)
                    goto offensiveTurn;
            }
        }
        action
        {
            lookLeftAndRight();
            if(theTeamBallModel.position.x()<0) {
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),std::get<0>(theLibCodeRelease.getOwnFieldOffSideKickPos()));
            }
            else{
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),std::get<0>(theLibCodeRelease.getOppFieldOffSideKickPos()));
            }
            
        }
    }

    state(offensiveTurn)
    {
        transition
        {
            if (theTeamBallModel.position.x()<0) {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
            }
            else {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::supporter || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
            }

            if(theRole.role == Role::RoleType::jolly){
                if((std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0))) < 10.f))
                    goto offensiveStand;
            }
            else {
                if((std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 10.f))
                    goto offensiveStand;
            }
        }
        action
        {
            LookForward();
            if(theRole.role == Role::RoleType::jolly){
                if(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0)) > 0.f)
                    WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
                else
                    WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));
            }
            else {
                if(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y())) > 0.f)
                    WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
                else
                    WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));
            }
        }
    }

    state(offensiveStand)
    {
        action
        {
            if (theTeamBallModel.position.x()<0) {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
            }
            else {
                if(theRole.role == Role::RoleType::defender || theRole.role == Role::RoleType::supporter || theRole.role == Role::RoleType::goalie)
                    goto sideKick;
            }

            if(theRole.role == Role::RoleType::jolly || theRole.role == Role::RoleType::supporter){
                Stand();
                if(state_time > 5000)
                    lookAtGlobalBall();
                else
                    lookLeftAndRight();
            }
            else if(theRole.role == Role::RoleType::striker){
                lookAtBall();
                std::tuple<Vector2f,Vector2f,bool> thePositions = theLibCodeRelease.getOppFieldOffSideKickPos();
                if(std::get<2>(thePositions)){
                    ChinaApproacher(true, thePossiblePlan.betterTarget);
                }
                else {
                    ChinaApproacher(true, std::get<1>(thePositions));
                }
            }
        }
    }

    state(ownFieldDefSideKick)
    {
        transition
        {
            if(theTeamBallModel.position.x() > 0 || theRole.role == Role::RoleType::jolly)
                goto sideKick;
            
            if((theRobotPose.translation-theLibCodeRelease.getOwnFieldDefSideKickPos()).norm() < 150.f)
                goto turnToBall;
        }

        action
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getOwnFieldDefSideKickPos());
        }
    }

    state(oppFieldDefSideKick)
    {
        transition
        {
            if(theTeamBallModel.position.x() < 0 || theRole.role == Role::RoleType::defender)
                goto sideKick;

            if((theRobotPose.translation-theLibCodeRelease.getOppFieldDefSideKickPos()).norm() < 150.f)
                goto turnToBall;
        }

        action
        {
                lookLeftAndRight();
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getOppFieldDefSideKickPos());
        }
    }

    state(turnToBall)
    {
        transition
        {
            if(state_time > 2000)
                goto sideKick;

            if((std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y()))) < 10.f))
                goto stand;
        }
        action
        {
            LookForward();
            if(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theTeamBallModel.position.x(),theTeamBallModel.position.y())) > 0.f)
                WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
            else
                WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));
        }
    }

    state(stand)
    {
        transition
        {
            if(state_time > 2000)
                goto sideKick;
        }
        action
        {
            Stand();
            if(state_time > 5000)
                lookLeftAndRight();
            else
                lookAtGlobalBall();
        }
    }
}