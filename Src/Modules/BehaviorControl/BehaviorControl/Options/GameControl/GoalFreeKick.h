option(GoalFreeKick)
{
    initial_state(goalFreeKick)
    {
        transition
        {
            if((int)theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
                goto offensiveGoalFreeKick;
           else
                goto defensiveGoalFreeKick;
        }
    }

    state(offensiveGoalFreeKick)
    {
        transition
        {
            if(theRobotInfo.number == 1)
                goto goalieTurnToBall;
            if(theRobotInfo.number == 3)
                goto defenderPos;
            if(theRobotInfo.number == 4)
                goto supporterPos;
            if(theRobotInfo.number == 2)
                goto strikerPos;
            if(theRobotInfo.number == 5)
                goto jollyPos;
        }
    }

    state(goalieTurnToBall)
    {
        transition
        {
            if((std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(),theBallModel.estimate.position.y()))) < 10.f))
                goto waitToMove;
        }
        action
        {
            if(theTeamBallModel.position.y() > 0)
                lookLeftAndRight();
            else
                lookRightAndLeft();
            if(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theBallModel.estimate.position.x(),theBallModel.estimate.position.y())) > 0.f)
                WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
            else
                WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));\
        }
    }

    state(strikerPos){
        transition
        {
            if((theRobotPose.translation - Vector2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosLeftSideline*2/3)).norm() < 150.f)
                goto turnToSide;
        }
        action
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),Pose2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosLeftSideline*2/3));
        }
    }

    state(jollyPos){
        transition
        {
            if((theRobotPose.translation - Vector2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosRightSideline*2/3)).norm() < 150.f)
                goto turnToSide;
        }
        action
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),Pose2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosRightSideline*2/3));
        }
    }

    state(defenderPos){
        transition
        {
            Vector2f globalBallPos = theTeamBallModel.position;
            if((theRobotPose.translation - Vector2f(globalBallPos.x(),globalBallPos.y() - 800)).norm() < 150.f)
                goto turnToBall;
        }
        action
        {
            Vector2f globalBallPos = theTeamBallModel.position;
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),Pose2f(globalBallPos.x(),globalBallPos.y() - 800));
        }
    }

    state(supporterPos){
        transition
        {
            Vector2f globalBallPos = theTeamBallModel.position;
            if((theRobotPose.translation - Vector2f(globalBallPos.x(),globalBallPos.y() + 800)).norm() < 150.f)
                goto turnToBall;
        }
        action
        {
            Vector2f globalBallPos = theTeamBallModel.position;
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),Pose2f(globalBallPos.x(),globalBallPos.y() + 800));
        }
    }

    state(turnToSide){
        transition
        {
            if(theRobotPose.translation.y() < 0.f){
                if(theRobotPose.rotation.toDegrees() > (float)86.5 && theRobotPose.rotation.toDegrees() < (float)93.5)
                    goto stand;
            }
            else{
                if(theRobotPose.rotation.toDegrees() < (float)-86.5 && theRobotPose.rotation.toDegrees() > (float)-93.5)
                    goto stand;
            }
        }
        action
        {
            if(theBallModel.estimate.position.norm() < 3000.f)
                lookAtBall();
            else
                lookAtGlobalBall();
            if(theRobotPose.translation.y() < 0.f){
                if(theRobotPose.rotation.toDegrees() >= -90.f && theRobotPose.rotation.toDegrees() < (float)89.5)
                    WalkAtRelativeSpeed(Pose2f(1.f,0.f,0.f));
                else
                    WalkAtRelativeSpeed(Pose2f(-1.f,0.f,0.f));
                }
            else{
                if(theRobotPose.rotation.toDegrees() >= (float)-89.5 && theRobotPose.rotation.toDegrees() <= 90.f)
                    WalkAtRelativeSpeed(Pose2f(-1.f,0.f,0.f));
                else
                    WalkAtRelativeSpeed(Pose2f(1.f,0.f,0.f));
            }
        }
    
    }

    state(waitToMove){
        transition
        {
            if(state_time > 5000)
                goto prepareForKick;
        }
        action
        {
            if(theTeamBallModel.position.y() > 0)
                lookLeftAndRight();
            else
                lookRightAndLeft();
            Stand();
        }
    }

    state(prepareForKick){
        transition
        {
            if(state_time > 8000 && theBallModel.estimate.position.norm() < 700.f)
                goto checkPos;
        }
        action
        {
            lookAtBall();
            ChinaApproacher(false, Pose2f(theFieldDimensions.xPosOpponentGroundline,0));
        }
    }

    state(checkPos)
    {
        transition
        {

            Vector2f pos1(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosLeftSideline*2/3-500); //left wing
            Vector2f pos2(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosRightSideline*2/3+500); //right wing
            
            for(const auto& someCorridor : theFreeCorridors.corridors){
                if(someCorridor.line.distance(pos2) < someCorridor.threshold)
                    goto kickTo2;
            }

            for(const auto& someCorridor : theFreeCorridors.corridors){
                if(someCorridor.line.distance(pos1) < someCorridor.threshold)
                    goto kickTo1;
            }
            
            goto kickCorr;
        }
    }

    state(kickTo1)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() > 2000.f)
                goto stand;
        }
        action
        {
            lookAtBall();
            ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosLeftSideline*2/3 - 500));
        }
    }

    state(kickTo2)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() > 2000.f)
                goto stand;
        }
        action
        {
            lookAtBall();
            ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline/8,theFieldDimensions.yPosRightSideline*2/3 + 500));
        }
    }

    state(kickCorr)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() > 2000.f)
                goto stand;
        }

        action
        {
            lookAtBall();
            if(theFreeCorridors.corridors.empty())
                ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline,0));
            else
                ChinaApproacher(true, theFreeCorridors.corridors[theFreeCorridors.maxIndex].target);
        }
    }

    state(turnToBall)
    {
        transition
        {
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
        action
        {
            Stand();
            if(theBallModel.estimate.position.norm() > 3000)
                lookAtGlobalBall();
            else
                lookAtBall();
        }
    }

    state(defensiveGoalFreeKick)
    {
        action
        {
            //TODO PRESSING POSITION
            Pose2f readyPose(theLibCodeRelease.getReadyPose(true, theRole.role ));
            if(theRole.role == Role::striker){
                lookAtBall();
                WalkToTargetPathPlanner(Pose2f(0.9f,0.9f,0.9f), Pose2f(theTeamBallModel.position)- Pose2f(750.f, 0.f));
            }
            else
                GetIntoReadyPosition(Pose2f( .6f, 70.f, 20.f), readyPose.translation );
        }
    }
}