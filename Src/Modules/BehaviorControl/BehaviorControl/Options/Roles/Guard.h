option(Guard, (const int) guardNumber)
{
    int guardXpose, guardYpose;

    if (guardNumber == 1) {
        //guardXpose = -3300; guardYpose = -1200;
        guardXpose = theLibCodeRelease.defenderPosition.x(); guardYpose = theLibCodeRelease.defenderPosition.y();
    }
    else if (guardNumber == 2){
        //guardXpose = -3300; guardYpose = 1200;
        guardXpose = theLibCodeRelease.supporterPosition.x(); guardYpose = theLibCodeRelease.supporterPosition.y();
    }

    else if (guardNumber == 4){
        //guardXpose = -3300; guardYpose = 1200;
        guardXpose = SPQR::GUARD_X_POSE; 
        guardYpose = SPQR::GUARD_Y_POSE;
    }


    initial_state(start)
    {
        transition
        {
            //std::cout<<"GUARD  "<< guardNumber <<  "   "<< "ANGLEE  "<< theRobotPose.rotation<<std::endl;

            if((theLibCodeRelease.norm(theRobotPose.translation.x()-guardXpose, theRobotPose.translation.y()-guardYpose)<300))
            {
                if(guardNumber == 1 && theRobotPose.rotation < 0.8  && theRobotPose.rotation > 0.6)
                    goto stand;
                else if (guardNumber == 2 && theRobotPose.rotation > -0.8  && theRobotPose.rotation < -0.6)
                    goto stand;
                else if(guardNumber == 4 && std::abs(theRobotPose.rotation)< 0.2)
                    goto stand;
            }

        }
        action
        {
            lookLeftAndRight();
            Stand();
            // if(guardNumber == 1)
            //     WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(Angle::fromDegrees(-10.f),(theLibCodeRelease.glob2Rel(guardXpose, guardYpose)).translation.x(), (theLibCodeRelease.glob2Rel(guardXpose, guardYpose)).translation.y() ));
            // if(guardNumber == 2)
            //     WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(Angle::fromDegrees(10.f), (theLibCodeRelease.glob2Rel(guardXpose, guardYpose)).translation.x(), (theLibCodeRelease.glob2Rel(guardXpose, guardYpose)).translation.y() ));

            if(guardNumber == 1)
                WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(Angle::fromDegrees(-15.f),guardXpose, guardYpose ));
            if(guardNumber == 2)
                WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(Angle::fromDegrees(15.f), guardXpose, guardYpose));
            if(guardNumber == 4)
                WalkToTargetPathPlanner(Pose2f(50.f, 50.f, 50.f), Pose2f(Angle::fromDegrees(10.f), guardXpose, guardYpose));

            //Arms back
            theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
            theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
        }
    }

    state(stand)
    {
        transition
        {
//            if(std::abs(theRobotPose.rotation) > Angle::fromDegrees(5.f))
//                goto turnToOpponentGoal;
        }
        action
        {
            Stand();
            //if(theRobotPose.rotation < )
            (theBallModel.lastPerception.y() > 0) ?
                            Esorcista(1) : Esorcista(-1);   // only head move
            
        }
    }
}
