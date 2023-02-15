#define SUPPORTER_MOVE_THRESHOLD 250
#define SUPPORTER_WALK_THRESHOLD 900

option(SupporterAU)
{
    initial_state(stand)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker;

            if((theRobotPose.translation-theLibCodeRelease.getSupporterPosition()).norm() > SUPPORTER_MOVE_THRESHOLD)
                goto move;
        }
        action 
        {
            if(state_time < 7000){
                Stand();
                lookLeftAndRight();
            }
            else{
                Turn360();
            }
        }
    }
    
    state(moveBehindStriker)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theTeamBallModel.position.x() < 2000)
                goto move;
            if(theRobotPose.translation.x() < strikerPosition.x())
                goto stand;
        }
        action 
        {
            if(theTeamBallModel.position.x() >= 2000)
                GoBehindStriker();
        }
    }

    state(move)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker;

            if((theRobotPose.translation-theLibCodeRelease.getSupporterPosition()).norm() < 100){
               if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 3){
                    if(theTeamBallModel.position.x() < 0.f || theLibCodeRelease.opponentOnOurField())
                        goto turnToOpponent;
                    else {
                        goto turnToBall;
                    }
                }
                else{
                    if(theTeamBallModel.position.x() < 0.f || !theLibCodeRelease.opponentOnOurField())
                        goto turnToBall;
                    else 
                        goto turnToOpponent;
                }
            }

            Vector2f targetPos = theLibCodeRelease.getJollyPosition();
            Vector2f relTargetPos = theLibCodeRelease.glob2Rel(targetPos.x(),targetPos.y()).translation;

            // Check if there's an obstacle in front of the robot
            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() > 0.f){
                    if(obs.center.x() < relTargetPos.x())
                        goto moveAroundObstacle;
                }
            }

        }
        action 
        {
            if((theRobotPose.translation-theLibCodeRelease.getSupporterPosition()).norm() > SUPPORTER_WALK_THRESHOLD){
                lookLeftAndRight();
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getSupporterPosition());
            }
            else {
                LookForward();
                Vector2f targetPosition = theLibCodeRelease.getSupporterPosition();
                Vector2f relTargetPos = theLibCodeRelease.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
                WalkToTarget(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
            }
        }
    }

    state(moveAroundObstacle)
    {
        transition
        {
            if((theRobotPose.translation-theLibCodeRelease.getSupporterPosition()).norm() < 100.f){
                if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 3){
                    if(theTeamBallModel.position.x() < 0.f || theLibCodeRelease.opponentOnOurField())
                        goto turnToOpponent;
                    else {
                        goto turnToBall;
                    }
                }
                else{
                    if(theTeamBallModel.position.x() < 0.f || !theLibCodeRelease.opponentOnOurField())
                        goto turnToBall;
                    else 
                        goto turnToOpponent;
                }
            }
        }
        action 
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getSupporterPosition());
        }
    }

    state(turnToOpponent)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker;

            Vector2f interCalcPosition = theLibCodeRelease.getSupporterPosition() - strikerPosition;
            Vector2f interCalc2Position = Vector2f(interCalcPosition.x()*1.3,interCalcPosition.y()*1.3);
            Vector2f targetPosition = strikerPosition + interCalc2Position;

            if(theRobotPose.translation.y() > strikerPosition.y()){
                // face topright-right
                float angleToTarget = std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(targetPosition.x(),targetPosition.y())));
                if(angleToTarget < 80.f && angleToTarget > 35.f && theRobotPose.rotation.toDegrees() > -90.f && theRobotPose.rotation.toDegrees() < 135.f)
                    goto stand;
            }
            else{
                //face bottomright-right
                float angleToTarget = std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(targetPosition.x(),targetPosition.y())));
                if(angleToTarget < 80.f && angleToTarget > 35.f && theRobotPose.rotation.toDegrees() < 90.f && theRobotPose.rotation.toDegrees() > -135.f)
                    goto stand;
            }
        }
        action
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }

            Vector2f interCalcPosition = theLibCodeRelease.getSupporterPosition();
            Vector2f interCalc2Position = Vector2f((interCalcPosition.x() - strikerPosition.x())*1.3,(interCalcPosition.y() - strikerPosition.y())*1.3);
            Vector2f targetPosition = strikerPosition + interCalc2Position;

            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::targetMode;
            Pose2f relLookPos = theLibCodeRelease.glob2Rel(targetPosition.x(),targetPosition.y());
            theHeadMotionRequest.target = Vector3f(relLookPos.translation.x(),relLookPos.translation.y(),20.f);
            theHeadMotionRequest.speed = 1;

            if(theRobotPose.translation.y() > strikerPosition.y()){
                // face topright-right
                WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));
            }
            else{
                //face bottomright-right
                WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
            }                
        }
    }

    state(turnToBall)
    {
        transition
        {   
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                }
            }
            if(theRobotPose.translation.x() > strikerPosition.x() && theTeamBallModel.position.x() > 2000)
                goto moveBehindStriker;

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
}
