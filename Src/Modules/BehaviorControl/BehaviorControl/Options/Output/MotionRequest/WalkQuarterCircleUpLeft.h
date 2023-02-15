option(WalkQuarterCircleUpLeft, (const float&) TurnRate)
{

    initial_state(start){
        transition{
            goto turning;
        }
    }
    
    state(turning)
    {
        transition
        {
            if(theRobotPose.rotation.toDegrees() > 85.5 && theRobotPose.rotation.toDegrees() < 94.5)
                goto moving;
        }

        action
        {
            if(theRobotPose.rotation.toDegrees() >= -90 && theRobotPose.rotation.toDegrees() < 89.5)
                WalkAtRelativeSpeed(Pose2f(1,0,0));
            else
                WalkAtRelativeSpeed(Pose2f(-1,0,0));
        }
    }

    state(moving)
    {
        transition
        {
            if(theRobotPose.rotation.toDegrees() > 175 || theRobotPose.rotation.toDegrees() < -175)
                goto targetState;
        }
        action
        {
            WalkAtRelativeSpeed(Pose2f(TurnRate,1,0));
        }
    }

    target_state(targetState)
    {
        transition
        {
            ;
        }
        action
        {
            Stand();
        }
    }
}