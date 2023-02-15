#define X_THRESHOLD 30
#define Y_THRESHOLD 20

// sidekick = 2100s
// sidewalk = 1250s

// Distance used for choosing kicks
//#define DISTANCE_THRESHOLD 1615

#define X_POS_STRONG 250
#define X_POS_FAST 200
#define Y_POS 62

#define SIDEKICK_THRESHOLD 30
#define X_POS_SIDE 0
#define Y_POS_SIDE 170

option(FastApproacher, (const Vector2f) theTarget, (const bool) beFast)
{
    initial_state(init)
    {
        transition
        {

            // if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 2000)
            //     goto findBall;

            if(theTeamBallModel.position.x() > theLibCodeRelease.rel2Glob(theTarget.x(),theTarget.y()).translation.x())
                goto init;

            // if between ball and target
            if(theBallModel.estimate.position.x() * theTarget.x() < 0 && theBallModel.estimate.position.x() < -300)
                goto pathToBall;

            Vector2f ballPoint = theBallModel.estimate.position;
            std::cout << theLibCodeRelease.radiansToDegree(atan2f(ballPoint.y(), ballPoint.x())) << " " <<
            theLibCodeRelease.radiansToDegree(atan2f(theTarget.y(), theTarget.x())) << std::endl;

            goto goNearBall;
        }
    }

    state(goNearBall){
        transition
        {
            if(theBallModel.estimate.position.norm() < 1500){

                Vector2f ballPoint = theBallModel.estimate.position;

                if(theLibCodeRelease.radiansToDegree(atan2f(ballPoint.y(), ballPoint.x())) < -55.f
                && theLibCodeRelease.radiansToDegree(atan2f(ballPoint.y(), ballPoint.x())) > -200.f
                && theLibCodeRelease.radiansToDegree(atan2f(theTarget.y(), theTarget.x())) < 0.f)
                    goto prepSKRight;

                if(theLibCodeRelease.radiansToDegree(atan2f(ballPoint.y(), ballPoint.x())) > 55.f
                && theLibCodeRelease.radiansToDegree(atan2f(ballPoint.y(), ballPoint.x())) < 200.f
                && theLibCodeRelease.radiansToDegree(atan2f(theTarget.y(), theTarget.x())) > 0.f)
                    goto prepSKLeft;

                Vector2f knownPoint = Vector2f(0.f,10.f); // a known point on the left
                float d = ballPoint.x() * theTarget.y() - ballPoint.y() * theTarget.x();
                float e = knownPoint.x() * theTarget.y() - knownPoint.y() * theTarget.x();
                // if same sign, this will be positive, which means the ball is on the left
                if(d*e > 0){
                    goto prepKL;
                }
                else{
                    goto prepKR;
                }
            }

        }
        action
        {
            Pose2f ballpoint;
            if(theTeamBallModel.isValid)
                ballpoint = Pose2f(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            else
                ballpoint = Pose2f(theBallModel.lastPerception.x(),theBallModel.lastPerception.y());
            lookLeftAndRight();
            if(theBallModel.estimate.position.norm() > 1500)
                WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),ballpoint);
        }
    }

    state(pathToBall){
        transition
        {
            if(theBallModel.estimate.position.x() * theTarget.x() > 0)
                goto init;
            if(state_time > 5000)
                goto init;
        }
        action
        {
            Pose2f ballpoint;
            if(theTeamBallModel.isValid)
                ballpoint = Pose2f(theTeamBallModel.position.x(),theTeamBallModel.position.y());
            else
                ballpoint = Pose2f(theBallModel.lastPerception.x(),theBallModel.lastPerception.y());
            lookLeftAndRight();
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f),ballpoint);
        }
    }

    state(prepKL)
    {
        transition
        {
            if(state_time > 3000)
                goto init;
            Vector2f ballPoint = theBallModel.estimate.position;
            if(!beFast) {
                if(ballPoint.x() < X_POS_STRONG + X_THRESHOLD && ballPoint.x() > X_POS_STRONG - X_THRESHOLD && ballPoint.y() > Y_POS - Y_THRESHOLD && ballPoint.y() < Y_POS + Y_THRESHOLD)
                        goto kickLeft_MUCH_WOW_VERY_POWER;
            }
            else {
                if(ballPoint.x() < X_POS_FAST + X_THRESHOLD*1.5f && ballPoint.x() > X_POS_FAST - X_THRESHOLD*1.5f && ballPoint.y() > Y_POS - Y_THRESHOLD*1.5f && ballPoint.y() < Y_POS + Y_THRESHOLD*1.5f)
                    goto kickLeft_MUCH_FAST_VERY_PEW_PEW;
            }
        }
        action
        {
            Vector2f ballPoint = theBallModel.estimate.position;
            lookAtBall();
            if(!beFast)
                WalkToTarget(Pose2f(1.f,.9f,9.f),Pose2f(atan2f(theTarget.y(), theTarget.x()),ballPoint.x()-X_POS_STRONG,ballPoint.y()-Y_POS));
            else
                WalkToTarget(Pose2f(1.f,.9f,9.f),Pose2f(atan2f(theTarget.y(), theTarget.x()),ballPoint.x()-X_POS_FAST,ballPoint.y()-Y_POS));
        }
    }

    state(prepKR)
    {
        transition
        {
            if(state_time > 3000) {
                goto init;
            }
            Vector2f ballPoint = theBallModel.estimate.position;
            if(!beFast) {
            if(ballPoint.x() < X_POS_STRONG + X_THRESHOLD && ballPoint.x() > X_POS_STRONG - X_THRESHOLD && ballPoint.y() > -Y_POS - Y_THRESHOLD && ballPoint.y() < -Y_POS + Y_THRESHOLD)
                goto kickRight_MUCH_WOW_VERY_POWER;
            }
            else {
            if(ballPoint.x() < X_POS_FAST + X_THRESHOLD*1.5f && ballPoint.x() > X_POS_FAST - X_THRESHOLD*1.5f && ballPoint.y() > -Y_POS - Y_THRESHOLD*1.5f && ballPoint.y() < -Y_POS + Y_THRESHOLD*1.5f)
                goto kickRight_MUCH_FAST_VERY_PEW_PEW;
            }
        }
        action
        {
            Vector2f ballPoint = theBallModel.estimate.position;
            lookAtBall();
            if(!beFast)
                WalkToTarget(Pose2f(1.f,.9f,9.f),Pose2f(atan2f(theTarget.y(), theTarget.x()),ballPoint.x()-X_POS_STRONG,ballPoint.y()+Y_POS));
            else
                WalkToTarget(Pose2f(1.f,.9f,9.f),Pose2f(atan2f(theTarget.y(), theTarget.x()),ballPoint.x()-X_POS_FAST,ballPoint.y()+Y_POS));
        }
    }

    state(kickLeft_MUCH_FAST_VERY_PEW_PEW)
    {
        transition
        {
            if(state_time > 3500)
                goto init;
        }
        action
        {
            PlaySound("weeeee.wav");
            theHeadControlMode = HeadControl::lookAtBall;
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
    }

    state(kickLeft_MUCH_WOW_VERY_POWER)
    {
        transition
        {
            if(state_time > 1500)
                goto init;
        }
        action
        {
            PlaySound("Fluttershyyay.wav");
            theMotionRequest.kickRequest.dynPoints.clear();
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = true;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
            theMotionRequest.kickRequest.dynPoints.clear();
        }
    }

    state(kickRight_MUCH_FAST_VERY_PEW_PEW)
    {
        transition
        {
            if(state_time > 3500)
                goto init;
        }
        action
        {
            PlaySound("weeeee.wav");
            theHeadControlMode = HeadControl::lookAtBall;
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
    }

    state(kickRight_MUCH_WOW_VERY_POWER)
    {
        transition
        {
            if(state_time > 1500)
                goto init;
        }
        action
        {
            PlaySound("Fluttershyyay.wav");
            theMotionRequest.kickRequest.dynPoints.clear();
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = false;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
            theMotionRequest.kickRequest.dynPoints.clear();
        }
    }

    state(prepSKRight)
    {
        transition
        {
            if(state_time > 5000)
                goto init;

            Vector2f ballPoint = theBallModel.estimate.position;
            float angleToTar = theLibCodeRelease.radiansToDegree(atan2f(theTarget.y(), theTarget.x()));

            if(ballPoint.x() < SIDEKICK_THRESHOLD*2 && ballPoint.x() > - SIDEKICK_THRESHOLD*2 && ballPoint.y() > -Y_POS_SIDE - SIDEKICK_THRESHOLD && ballPoint.y() < -Y_POS_SIDE + SIDEKICK_THRESHOLD
               && angleToTar < -85.f && angleToTar > -95.f){
                if(!beFast)
                    goto sideKickRight_MUCH_WOW_VERY_POWER;
                else
                    goto sideKickRight_MUCH_FAST_VERY_PEW_PEW;
            }
        }
        action
        {
            Vector2f ballPoint = theBallModel.estimate.position;
            lookAtBall();

            float angleToTarget = atan2f(theTarget.y(), theTarget.x());
            float angleToFace = Angle::fromDegrees(90)+angleToTarget;
            WalkToTarget(Pose2f(1.f,1.f,1.f),Pose2f(angleToFace,ballPoint.x(),ballPoint.y()+Y_POS_SIDE));
        }
    }

    state(prepSKLeft)
    {
        transition
        {
            if(state_time > 5000)
                goto init;

            Vector2f ballPoint = theBallModel.estimate.position;
            float angleToTar = theLibCodeRelease.radiansToDegree(atan2f(theTarget.y(), theTarget.x()));

            if(ballPoint.x() < SIDEKICK_THRESHOLD*2 && ballPoint.x() > - SIDEKICK_THRESHOLD*2 && ballPoint.y() > Y_POS_SIDE - SIDEKICK_THRESHOLD && ballPoint.y() < Y_POS_SIDE + SIDEKICK_THRESHOLD
                && angleToTar > 85.f && angleToTar < 95.f){
                if(!beFast)
                    goto sideKickLeft_MUCH_WOW_VERY_POWER;
                else
                    goto sideKickLeft_MUCH_FAST_VERY_PEW_PEW;
            }

        }
        action
        {
            Vector2f ballPoint = theBallModel.estimate.position;
            lookAtBall();

            float angleToTarget = atan2f(theTarget.y(), theTarget.x());
            float angleToFace = -Angle::fromDegrees(90)+angleToTarget;
            WalkToTarget(Pose2f(1.f,1.f,1.f),Pose2f(angleToFace,ballPoint.x(),ballPoint.y()-Y_POS_SIDE));
        }
    }

    state(sideKickRight_MUCH_WOW_VERY_POWER)
    {
        transition
        {
            if(state_time > 2200)
                goto init;
        }
        action
        {
            PlaySound("Fluttershyyay.wav");
            theMotionRequest.kickRequest.dynPoints.clear();
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = false;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::sideKick;
            theMotionRequest.kickRequest.dynPoints.clear();
        }
    }

    state(sideKickRight_MUCH_FAST_VERY_PEW_PEW)
    {
        transition
        {
            if(state_time > 1350)
                goto init;
        }
        action
        {
            PlaySound("weeeee.wav");
            theHeadControlMode = HeadControl::lookAtBall;
            InWalkKick(WalkKickVariant(WalkKicks::sideKick, Legs::right), Pose2f(0.f, 0.f, 0.f));
        }
    }

    state(sideKickLeft_MUCH_WOW_VERY_POWER)
    {
        transition
        {
            if(state_time > 2200)
                goto init;
        }
        action
        {
            PlaySound("Fluttershyyay.wav");
            theMotionRequest.kickRequest.dynPoints.clear();
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = true;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::sideKick;
            theMotionRequest.kickRequest.dynPoints.clear();
        }
    }

    state(sideKickLeft_MUCH_FAST_VERY_PEW_PEW)
    {
        transition
        {
            if(state_time > 1350)
                goto init;
        }
        action
        {
            PlaySound("weeeee.wav");
            theHeadControlMode = HeadControl::lookAtBall;
            InWalkKick(WalkKickVariant(WalkKicks::sideKick, Legs::left), Pose2f(0.f, 0.f, 0.f));
        }
    }

}
