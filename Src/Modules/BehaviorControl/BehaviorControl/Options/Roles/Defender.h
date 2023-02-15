#define DO_TURN_THRESHOLD 1080.f //si girano quando si allontanano di un metro dalla posizione
#define TOLERANCE_DEGREES_FOR_TURN 10_deg
#define STATIC_THRESHOLD 400.f
#define NOT_MOVING_THRESHOLD -1000.f
#define TURN_TO_BALL_THRESHOLD 20.f

#define WAIT_TEAMMATES

float radiusDef;

option(Defender)
{

//    common_transition
//    {
//        if(((int)theGameInfo.setPlay == 1 && (int)theGameInfo.kickingTeam==19) ||
//                (int)theGameInfo.setPlay == 2 && (inttheGameInfo.kickingTeam==!19)){

//            if(theTeamBallModel.position.norm() < 900)

//        }
//    }
    initial_state(start)
    {
        transition
        {
            radiusDef = 1000.f;
            goto turnToPose;
        }
        action
        {
//            Stand();
            WalkToTarget(Pose2f(50.f, 50.f, 50.f),
                         Pose2f(theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(),
                                                           theLibCodeRelease.defenderPosition.y())));
            LookForward();
        }
    }

    state(turnToPose)
    {
        transition
        {
            if(std::abs( theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleForDefender) ) < 10.f )
                goto walkToPose;
        }
        action
        {
            if (state_time > 500.f){
                if((theLibCodeRelease.angleForDefender) > 0.f)
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
//            if(/*(std::abs(theLibCodeRelease.angleForDefender) > TOLERANCE_DEGREES_FOR_TURN) &&*/
//                    (theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() > DO_TURN_THRESHOLD)
//                goto turnToPose;

#ifdef WAIT_TEAMMATES
            for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
                if((theTeamData.teammates.at(i).theRobotPose.translation -
                        theRobotPose.translation).norm() < (radiusDef)){
                    radiusDef -= 10.f;
                    goto stand;
                }
            }
#endif

            if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() < STATIC_THRESHOLD)
                goto wait;
        }
        action
        {
            lookLeftAndRight();
            if((theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(),
                                           theLibCodeRelease.defenderPosition.y())).translation.norm() < 3*STATIC_THRESHOLD){

                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theLibCodeRelease.glob2Rel(theLibCodeRelease.defenderPosition.x(),
                                                                                     theLibCodeRelease.defenderPosition.y())));
            } else {
                WalkToTargetPathPlanner(Pose2f(.8f, .8f, .8f), Pose2f(theLibCodeRelease.defenderPosition.x(),
                                                              theLibCodeRelease.defenderPosition.y()));
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
            radiusDef = 1000.f;
            if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
                if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() > 3*STATIC_THRESHOLD)
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

            if (iAmMostNearPlayer) {
                Approacher(true,Pose2f(0.f, (float)theFieldDimensions.xPosOpponentGroundline, 0.f));
            }
        }
    }


////    state(walkToBall)
////    {
////        transition
////        {
////            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut ||
////                    theBallModel.estimate.position.norm() > 1000.f)
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
//            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut ||
//                    theBallModel.estimate.position.norm() > 1000.f)
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
//            std::cout<< "I'm the Defender and I am waiting!"<< std::endl;
            if (theTeamBallModel.position.x() < NOT_MOVING_THRESHOLD || 1){
                if((theRobotPose.translation - theLibCodeRelease.defenderPosition).norm() > STATIC_THRESHOLD)
                    goto walkToPose;
                else
                    goto wait;
            }
        }
        action{
            Stand();
        }
    }
}
