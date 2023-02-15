Pose2f targetCKC;// = {0.f, 0.f, 0.f};
option(StrikerCornerKickChallenge)
{

    common_transition
    {
        if ((theKeyStates.pressed[KeyStates::headFront]))
            goto oneArm;
        else if ((theKeyStates.pressed[KeyStates::headMiddle]))
            goto twoArms;
        else if ((theKeyStates.pressed[KeyStates::headRear]))
            goto armOnTheSide;
    }

    initial_state(start)
    {
        transition
        {
//            goto twoArms;

            if(state_time > 7000)
                goto turnToBall;


            if ((theKeyStates.pressed[KeyStates::headFront]))
                goto oneArm;
            else if ((theKeyStates.pressed[KeyStates::headMiddle]))
                goto twoArms;
            else if ((theKeyStates.pressed[KeyStates::headRear]))
                goto armOnTheSide;
        }
        action
        {

            lookLeftAndRight();
            //            if (state_time > 3000 && state_time < 12000){
            //            } else {
            //                Stand();
            //            }
            //            if (state_time > 9500)
            //                lookLeftAndRight();
        }
    }

    state (oneArm) {
        transition
        {
            if(state_time > 5000)
                goto walkToBall;
        }
        action {
            CommunicateWithGestures("oneArm");
        }
    }

    state (twoArms) {
        transition
        {
            if(state_time > 13000)
                goto turnToBall;
        }
        action {
            CommunicateWithGestures("twoArms");
        }
    }

    state (armOnTheSide) {
        transition
        {
            if(state_time > 10000)
                goto turnToBall;
        }
        action {
            CommunicateWithGestures("armOnTheSide");
        }
    }

    state(turnToBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
                goto oneArm;
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
//            WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(walkToBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;
            if(theBallModel.estimate.position.norm() < 500.f){
                goto passBasedOnArms;
            }
//            goto passBasedOnArms;   //VINC PAY ATTENTION ON THIS
        }
        action
        {
            theHeadControlMode = HeadControl::lookAtBall;
            WalkToTarget(Pose2f(0.8f, .8f, .8f), theBallModel.estimate.position);
        }
    }

    state(alignToGoal)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;
            if(theRobotPose.translation.y()<0){
                if(theLibCodeRelease.between(theRobotPose.rotation, 1.9f, 2.1f) && std::abs(theBallModel.estimate.position.y()) < 500.f)
                    goto passBasedOnArms;
            }
            else{
                if(theLibCodeRelease.between(theRobotPose.rotation, -2.1f, -1.9f) && std::abs(theBallModel.estimate.position.y()) < 500.f)
                    goto passBasedOnArms;
            }
        }
        action
        {
            theHeadControlMode = HeadControl::lookForward;
            if(theRobotPose.translation.y()<0)
                WalkToTarget(Pose2f(.7f, .7f, .7f), Pose2f(0.8f, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
            else
                WalkToTarget(Pose2f(.7f, .7f, .7f), Pose2f(-0.8f, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
        }
    }

    state(passBasedOnArms)
    {
        transition
        {
            goto approacher;

            for(const auto& obs : theObstaclesFieldPercept.obstacles) {

                if(obs.type == ObstaclesFieldPercept::ownPlayer){
                    if (obs.obsPose == ObstaclesFieldPercept::oneArmRaised){
                        targetCKC = theLibCodeRelease.rel2Glob(obs.center.x(), obs.center.y()).translation;
                        goto approacher;
                    } else if (obs.obsPose == ObstaclesFieldPercept::twoArmsRaised){
                        targetCKC = theLibCodeRelease.rel2Glob(obs.center.x(), obs.center.y()).translation;
                        goto approacher;
                    }
                }
            }
        }
        action
        {
            lookLeftAndRight();
            Stand();

        }
    }

    state(approacher)
    {
        transition
        {
        if (state_time > 7000)
            goto stand;
        }
        action{
            //FastApproacher(theLibCodeRelease.glob2Rel(4500, 0).translation, false);
            //InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
            if (state_time > 500)
                //Kicks("veryFastForwardKick");
                StrikerDemo();
            //ChinaApproacher(true, Pose2f(4500, 0));
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
    state(stand){
        action{Stand();}
    }
}
