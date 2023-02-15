#define DO_TURN_THRESHOLD 1080.f
#define TOLERANCE_DEGREES_FOR_TURN 10_deg
#define STATIC_THRESHOLD 400.f
#define NOT_MOVING_THRESHOLD -1000.f
#define TURN_TO_BALL_THRESHOLD 20.f

#define WAIT_TEAMMATES

float radiusSupp;
option(Supporter)
{
    common_transition{
//        std::cout << "radiusSupp: "<< radiusSupp<< std::endl;
    }
    initial_state(start)
    {
        transition
        {
            radiusSupp = 1000.f;
            goto turnToPose;
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(50.f, 50.f, 50.f),
                         Pose2f(theLibCodeRelease.glob2Rel(theLibCodeRelease.supporterPosition.x(),
                                                           theLibCodeRelease.supporterPosition.y())));
        }
    }

    state(turnToPose)
    {
        transition
        {
            if(std::abs( theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleForSupporter) ) < DO_TURN_THRESHOLD )
                goto walkToPose;
        }
        action
        {
            if (state_time > 500.f){
                if((theLibCodeRelease.angleForSupporter) > 0.f)
                    WalkAtRelativeSpeed(Pose2f(20.f, 0.0001f,0.0001f));
                else
                    WalkAtRelativeSpeed(Pose2f(-20.f, 0.0001f,0.0001f));
            }
            else{
                Stand();
                lookLeftAndRight();
            }
        }
    }

    state(walkToPose)
    {
        transition
        {
//            if(/*(std::abs(theLibCodeRelease.angleForSupporter) > TOLERANCE_DEGREES_FOR_TURN) &&*/
//                    (theRobotPose.translation - theLibCodeRelease.supporterPosition).norm() > DO_TURN_THRESHOLD)
//                goto turnToPose;

//            for(int i = 0; i < theTeamData.teammates.size(); i++){
//                if((theTeamData.teammates.at(i).theRobotPose.translation -
//                        theLibCodeRelease.supporterPosition).norm() < (radiusSupp)){
//                    radiusSupp -= 10.f;
//                    goto stand;
//                }
//            }

#ifdef WAIT_TEAMMATES
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theRobotPose.translation).norm() < (radiusSupp)){
                    radiusSupp -= 10.f;
                    goto stand;
                }
            }
#endif

            if((theRobotPose.translation - theLibCodeRelease.supporterPosition).norm() < STATIC_THRESHOLD)
                goto wait;
        }
        action
        {
            lookLeftAndRight();
            if((theLibCodeRelease.glob2Rel(theLibCodeRelease.supporterPosition.x(),
                                           theLibCodeRelease.supporterPosition.y())).translation.norm() < 4*STATIC_THRESHOLD){          //controllare 800
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theLibCodeRelease.glob2Rel(theLibCodeRelease.supporterPosition.x(),
                                                                                     theLibCodeRelease.supporterPosition.y())));
            } else {
                WalkToTargetPathPlanner(Pose2f(50.f, 80.0f, 80.0f), Pose2f(theLibCodeRelease.supporterPosition.x(),
                                                              theLibCodeRelease.supporterPosition.y()));
                //Arms back
                theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
                theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
                theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
                theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
            }
        }
    }


    state(wait)
    {
        transition
        {
            if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
                if((theRobotPose.translation - theLibCodeRelease.supporterPosition).norm() > 4*STATIC_THRESHOLD)
                    goto walkToPose;

            }
        }

        action
        {
            //body
            if (std::abs(theLibCodeRelease.angleToGoal) > Angle::fromDegrees(TURN_TO_BALL_THRESHOLD)){
                WalkToTarget(Pose2f(50.f, 50.f, 50.f),
                             Pose2f(theLibCodeRelease.angleToGoal/*theBallModel.estimate.position.angle()*/, 0.f, 0.f));
            } else {
                Stand();
                radiusSupp = 1000.f;
//                Stopper();
            }

            //head
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
                lookAtBall();
            } else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
                lookAtGlobalBall();
            } else {
                (theBallModel.lastPerception.y() > 0) ?
                            Esorcista(1) : Esorcista(-1);
            }


            float myDistanceToBall = theBallModel.estimate.position.norm();
            bool iAmMostNearPlayer = true;
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theTeamBallModel.position).norm() < (myDistanceToBall+400.f) && theTeamData.teammates.at(i).number != 1){
                    iAmMostNearPlayer = false;
                }
            }


        }
    }

////    state(walkToBall)
////    {
////        transition
////        {
////            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
////                    || theBallModel.estimate.position.norm() > 1000.f)
////                goto turnToPose;
////            if(theBallModel.estimate.position.norm() < 500.f)
////                goto alignToGoal;
////        }
////        action
////        {
////            lookAtBall();
////            WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
////        }
////    }

//    state(alignToGoal)
//    {
//        transition
//        {
//            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut
//                    || theBallModel.estimate.position.norm() > 1000.f)
//                goto turnToPose;
//        }
//        action
//        {
//            lookAtBall();
//            GoAndKick(Pose2f(0.f, (float)theFieldDimensions.xPosOpponentGroundline, 0.f));
//        }
//    }

    state(stand){
        transition{
//            std::cout<< "I'm the Supporter and I am waiting!"<< std::endl;
            if((theRobotPose.translation - theLibCodeRelease.supporterPosition).norm() > STATIC_THRESHOLD)
                goto walkToPose;
            else
                goto wait;
        }
        action{
            Stand();
        }
    }
}
