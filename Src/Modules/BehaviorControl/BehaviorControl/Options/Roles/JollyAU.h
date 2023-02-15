#define JOLLY_MOVE_THRESHOLD 250
#define JOLLY_WALK_THRESHOLD 900

option(JollyAU)
{
    initial_state(stand)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((theRobotPose.translation-theLibCodeRelease.getJollyPosition()).norm() > JOLLY_MOVE_THRESHOLD)
                goto movePlan;
        }
        action
        {
            Stand();
            lookLeftAndRight();
        }
    }

    state(movePlan)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            Vector2f targetPos = theLibCodeRelease.getJollyPosition();
            Vector2f relTargetPos = theLibCodeRelease.glob2Rel(targetPos.x(),targetPos.y()).translation;

            // Check if there's an obstacle in front of the robot
            for(const auto& obs : theObstacleModel.obstacles){
                if(obs.center.x() < relTargetPos.x() && std::abs(obs.center.x()) < 800.f)
                    goto moveAroundObstacle;
            }

            if((theRobotPose.translation-targetPos).norm() < JOLLY_WALK_THRESHOLD)
                goto moveWalk;
        }
        action
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getJollyPosition());
        }
    }

    state(moveAroundObstacle)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((theRobotPose.translation-theLibCodeRelease.getJollyPosition()).norm() < 100.f)
                goto turnToBall;
        }
        action
        {
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),theLibCodeRelease.getJollyPosition());
        }
    }

    state(moveWalk)
    {
        transition
        {
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

            if((theRobotPose.translation-theLibCodeRelease.getJollyPosition()).norm() < 100.f)
                goto turnToBall;
        }
        action
        {
            LookForward();
            Vector2f targetPosition = theLibCodeRelease.getJollyPosition();
            Vector2f relTargetPos = theLibCodeRelease.glob2Rel(targetPosition.x(),targetPosition.y()).translation;
            WalkToTarget(Pose2f(1.f,1.f,1.f),Pose2f(relTargetPos.x(),relTargetPos.y()));
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
                    break;
                }
            }
            std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
            if(std::get<0>(strikerPS) == 1 && std::get<1>(strikerPS) == theRobotInfo.number
            && (std::get<2>(strikerPS).translation-strikerPosition).norm() > 1500 && std::get<2>(strikerPS).translation != Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f))
                goto receivePass;

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

    state(receivePass)
    {
        transition
        {
          // turn toward the goal but looking at the ball
          goto turnToGoal;
       }
        action
        {

        }
    }

    state(turnToGoal)
    {
        transition
        {
            if((std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0))) < 10.f))
                goto waitBall;
        }
        action
        {
          // get the angle the ball and the goal
          // initialize ball position to the target in case we don't know where the ball is
          std::tuple<int,int,Pose2f> strikerPS = theLibCodeRelease.strikerPassShare();
          Vector2f ballPos = std::get<2>(strikerPS).translation;
          // get the local ball if seen, otherwise use global
          if(theFrameInfo.time-theBallModel.timeWhenLastSeen < 500.f) {
            ballPos = theBallModel.estimate.position;
          } else if (theTeamBallModel.isValid) {
            ballPos = theTeamBallModel.position;
          }

          Vector2f goalPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f);
          /* float dx21 = x2-x1; */
          /* float dx31 = x3-x1; */
          /* float dy21 = y2-y1; */
          /* float dy31 = y3-y1; */
          /* float m12 = std::sqrt(dx21*dx21 + dy21*dy21); */
          /* float m13 = std::sqrt(dx31*dx31 + dy31*dy31); */
          /* float theta = std::cos((dx21*dx31 + dy21*dy31) / (m12 * m13)); */
          Pose2f angleBallGoal = Pose2f(goalPos.dot(ballPos)*0.25);

          lookAtBall();
          Turn(angleBallGoal);
          /* LookForward(); */
          /* if(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(theFieldDimensions.xPosOpponentGroundline,0)) > 0.f) */
          /*   WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f)); */
          /* else */
          /*   WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f)); */
        }
    }

   state(waitBall)
    {
        transition
        {
            // If too long and ball didn't reach, that means the pass failed
            Vector2f strikerPosition;
            for(const auto& teammate : theTeamData.teammates){
                if(teammate.role == Role::RoleType::striker) {
                    strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
                    break;
                }
            }
            if(state_time > 5000)
                goto stand;
        }
        action
        {
            Stand();
            lookAtBall();
        }
    }
}
