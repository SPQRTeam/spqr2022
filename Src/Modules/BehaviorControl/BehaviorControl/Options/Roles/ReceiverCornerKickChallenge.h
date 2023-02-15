Pose2f targetCKCRec;// = {0.f, 0.f, 0.f};
int singleArmCounter = 0;
int doubleArmCounter = 0;
#define ARMCOUNTER_THRES 40;
/** A test striker option without common decision */
option(ReceiverCornerKickChallenge)
{

    common_transition
    {

    }
    initial_state(start)
    {
        transition
        {
            //                goto turnToBall;
            if (state_time > 11000)
                goto backward;
        }
        action
        {
            Stand();
        }
    }

    state(turnToBall)
    {
        transition
        {
//            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
//                goto searchForBall;
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                goto percept;


        }
        action
        {
            SetHeadPanTilt(0.f, 0.25f, 150_deg);
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                Stand();
            else
                WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(percept)
    {
        transition
        {
//            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
//                goto searchForBall;
//            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
//                goto walkToBall;
            std::cout << "singleArmCounter "<< singleArmCounter << ", doubleArmCounter " << doubleArmCounter << "\n";
            for(const auto& obs : theObstaclesFieldPercept.obstacles) {

                if(obs.type == ObstaclesFieldPercept::ownPlayer){
                    if (obs.obsPose == ObstaclesFieldPercept::oneArmRaised){
                        targetCKCRec = theLibCodeRelease.rel2Glob(obs.center.x(), obs.center.y()).translation;
                        singleArmCounter++;
//                        goto walkToPosition_FirstPost;
                    } else if (obs.obsPose == ObstaclesFieldPercept::twoArmsRaised){
                        targetCKCRec = theLibCodeRelease.rel2Glob(obs.center.x(), obs.center.y()).translation;
                        doubleArmCounter++;
//                        goto walkToPosition_SecondPost;
                    }
                    if ((singleArmCounter+doubleArmCounter) > 6) {
                        if (singleArmCounter > doubleArmCounter)
                            goto walkToPosition_FirstPost;
                        else
                            goto walkToPosition_SecondPost;

                    }
                }
            }
        }
        action
        {
            SetHeadPanTilt(0.f, 0.15f, 150_deg);
            Stand();
        }
    }

    state(walkToPosition_FirstPost)
    {
        transition
        {
            if((theRobotPose.translation - Vector2f(4200.f, -600.f)).norm() < 500.f)
                goto turnToBall_stand;
            if(theBallModel.estimate.position.norm() < 700.f){
                goto approacher;
            }
        }
        action
        {
            CommunicateWithGestures("oneArm");
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTargetPathPlanner(Pose2f(0.8f, .8f, .8f), Pose2f(0.f, 4200.f, -600.f));
        }
    }

    state(walkToPosition_SecondPost)
    {
        transition
        {
            if((theRobotPose.translation - Vector2f(4200.f, 600.f)).norm() < 500.f)
                goto turnToBall_stand;
            if(theBallModel.estimate.position.norm() < 700.f){
                goto approacher;
            }
        }
        action
        {
            CommunicateWithGestures("twoArms");
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTargetPathPlanner(Pose2f(0.8f, .8f, .8f), Pose2f(0.f, 4200.f, 600.f));
        }
    }


    state(turnToBall_stand)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() < 700.f)
                goto approacher;
        }
        action
        {
            SetHeadPanTilt(0.f, 0.25f, 150_deg);
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                Stand();
            else
                WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }


    state(approacher)
    {
        transition
        {
            if (action_done)
                goto initialPose;
        }
        action{
            ChinaApproacher(true, Pose2f(4500.f, 0.f));
        }
    }

    state(initialPose)
    {
        transition
        {
            if((theRobotPose.translation - Vector2f(3900.f, -1550.f)).norm() < 500.f)
                    goto turnToBall_stand;
        }
        action{
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTargetPathPlanner(Pose2f(0.8f, .8f, .8f), Pose2f(0.f, 3900.f, -1550.f));
        }
    }

    state(searchForBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < 300)
                goto turnToBall;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
        }
    }

    state(backward){
        transition{if (state_time > 5000) goto stand;}
        action{
            lookAtBall();
            WalkAtRelativeSpeed(Pose2f(0.f, -1.f, 0.f));
        }
    }

    state(stand){
        action{
            lookAtBall();
            if(std::abs(theBallModel.estimate.position.norm()) < 1500){
//                ChinaApproacher(true, Pose2f(-4500, 0));
                StrikerDemo();
            } else{
                Stand();
            }
        }

    }
}
