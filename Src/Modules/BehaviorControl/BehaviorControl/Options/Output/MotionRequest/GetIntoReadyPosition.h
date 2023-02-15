
//#define DEBUG_READY_POSITIONING

option(GetIntoReadyPosition, (const Pose2f&) speed, (const Pose2f&) target)
{
    initial_state(start)
    {
        transition
        {
#ifdef DEBUG_READY_POSITIONING
            STATE("start");
#endif
        
        /* 
        if( theLibCodeRelease.norm(target.translation.x(), target.translation.y()) > 250)
            goto walkToReadyPosition;

        }
        */
            if(state_time > 1000)
                goto walkToReadyPosition; 
        }

        action
        {
            lookLeftAndRight();
            Stand();
        }
    }

    state(walkToReadyPosition)
    {
        transition
        {
#ifdef DEBUG_READY_POSITIONING
            STATE("walkToReadyPosition"):
#endif
/*
            std::cout<< " getintoreadyposition  X:  "<<theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(),
                                                  theLibCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.x(), 400)<<std::endl;
            std::cout<< " getintoreadyposition  Y:  "<<theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(),
                                                  theLibCodeRelease.rel2Glob(target.translation.x(), target.translation.y()).translation.y(), 400)<<std::endl;
            */

            if( theLibCodeRelease.isValueBalanced(theRobotPose.translation.x(),target.translation.x(), 400) &&
                    theLibCodeRelease.isValueBalanced(theRobotPose.translation.y(), target.translation.y(), 400) )
            {
                goto getReady;
            }
        }
        action
        {
            lookLeftAndRight();
           /*  if( theRobotPose.translation.x() > 500.f){
                if(std::abs(target.translation.angle()) > Angle::fromDegrees(10.f))
                    SPQRWalkTo(Pose2f(50.f, 80.0f, 80.0f), Pose2f(target.translation.angle(), 0.001f, 0.001f), target.translation.angle());

                else
                    SPQRWalkTo( Pose2f(80.0f, 80.0f, 80.0f), target , 0.f);
           }
            //else
            //{
                if(std::abs(target.translation.angle()) > Angle::fromDegrees(10.f))
                {
                    std::cout<< "STO IN IF    target: " << target.translation << std::endl;
                    WalkToTarget(Pose2f(50.f, 80.0f, 80.0f), Pose2f(target.translation.angle(), 0.001f, 0.001f));
                }
                else
                {
                    std::cout<< "STO IN ELSE  target: " << target.translation << std::endl;
                    SPQRWalkTo( Pose2f(80.0f, 80.0f, 80.0f), target , target.translation.angle());
                }*/

                                WalkToTargetPathPlanner( Pose2f(.8f, .8f, .8f), target);

        }
    }

    state(getReady)
    {
        transition
        {
#ifdef DEBUG_READY_POSITIONING
            STATE("getReady");
#endif
            if( theLibCodeRelease.between(theRobotPose.rotation,
                                          theLibCodeRelease.angleToGoal - Angle::fromDegrees(10.f) ,
                                          theLibCodeRelease.angleToGoal + Angle::fromDegrees( 10.f)  ) )
                goto finish;
        }
        action
        {
            lookLeftAndRight();
            if( theRobotPose.rotation > .0f )
                WalkAtRelativeSpeed(Pose2f(-50.f, 0.001f, 0.001f));
            else
                WalkAtRelativeSpeed(Pose2f(50.f, 0.001f, 0.001f));
        }
    }

    state(finish)
    {
        transition
        {

        }
        action
        {
            Stand();
            lookLeftAndRight();
        }
    }

}
