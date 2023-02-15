/** A test striker option without common decision */
Pose2f passTarget_S;
Pose2f passingMate_S;
bool isTheFirstTime;
Pose2f poseOfBetterTarget;


//#define STATIC_STRATEGY
#define PREAPPORACH_THRESHOLD 600.f
#define APPROACH_THE_BALL_THRESHOLD 300.f
#define RADIUS_TO_BACKWARD_PLACING 300.f

option(StrikerDev)
{

    initial_state(start)
    {
        transition
        {
            goto kick_off;
        }
        action
        {
            LookForward();
            Stand();
        }
    }

    state(kick_off)
    {
        transition
        {
            // no kick-off & the other team touches the ball or 10 seconds
            if( theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber ) {
                if( theTeamBallModel.position.norm() > 300.f || state_time > 10000 ){
#ifdef STATIC_STRATEGY
                    if (theTeamBallModel.position.norm() < 400.f)
                        goto passToJolly;
                    else goto turnToBall;
#else
                    goto turnToBall;
#endif
                }
            }
            // kick-off
            else{
#ifdef STATIC_STRATEGY
                if (theTeamBallModel.position.norm() < 400.f)
                    goto passToJolly;
                else goto turnToBall;
#else
                goto turnToBall;
#endif
            }
        }
        action
        {
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(7.f)){
                WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
            } else {
                Stand();
            }
            lookAtBall();
        }
    }


    state(turnToBall)
    {
        transition
        {
//            std::cout<<"turnToBall"<<std::endl;
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;
            if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(7.f))
                goto walkToBallDirect;
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }


    state(walkToBallDirect)
    {
        transition
        {
            theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
            theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;

            if (theBallModel.estimate.position.norm() < APPROACH_THE_BALL_THRESHOLD){
                goto approachTheBall;
            }


//            std::cout<<"walkToBallDirect"<<std::endl;
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;

/*
            for(int i = 0; i < theTeamData.teammates.size(); i++){
                if(theTeamData.teammates.at(i).thePassShare.myNumber == thePassShare.passingTo){
                    if(theTeamData.teammates.at(i).thePassShare.readyReceive == 1){
                        passingMate_S = theTeamData.teammates.at(i).theRobotPose;
                        goto executePass;
                    }
                }
            }
            */
        }
        action
        {
            lookAtBall();
            Pose2f ballGlob =  theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x() , theBallModel.estimate.position.y() ) ;
            SPQRWalkTo(Pose2f(50.f, 50.f, 50.f),
                       Pose2f(ballGlob.translation.x(), ballGlob.translation.y()  ), theLibCodeRelease.angleToGoal);
            isTheFirstTime = true;

            //Arms back
            theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
            theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
            theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;
        }
    }


    state(approachTheBall)
    {
        transition
        {

            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                goto searchForBall;
            //std::cout<<"A"<<std::endl;
            if(state_time > 10000 ||((state_time > 10 && action_done) && theBallModel.estimate.position.norm() >= 500) ){
                goto walkToBallDirect;
            }
            if( (state_time > 10 && action_done) && theBallModel.estimate.position.norm() < 500 ){
                goto walkToGoal;

            }
        }
        action
        {
            if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentDropInLine && std::abs(theRobotPose.translation.y())< 700.f )){
                ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline,theRobotPose.translation.y()));
                //std::cout<<"caso1"<<std::endl;

            }else{
                ChinaApproacher(false, Pose2f(0,0));
                //std::cout<<"caso2"<<std::endl;
            }

        }
    }

    state(walkToGoal)
    {
        transition
        {
//            std::cout<<"walkToGoal"<<std::endl;
            //std::cout<<"B"<<std::endl;
            if (theBallModel.estimate.position.x() < 0){
                goto approachTheBall;
            }
            if(theBallModel.estimate.position.norm() > 600){
                goto walkToBallDirect;
            }
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut/2)
                goto searchForBall;

            if(std::abs(theBallModel.estimate.position.y()) > 130.f ||
                    theBallModel.estimate.position.norm() > 400.f){

                goto turnToBall;
            }
/*
            else
            {
                for(int i = 0; i < theTeamData.teammates.size(); i++){
                    if(theTeamData.teammates.at(i).thePassShare.myNumber == thePassShare.passingTo){
                        if(theTeamData.teammates.at(i).thePassShare.readyReceive == 1){
                            passingMate_S = theTeamData.teammates.at(i).theRobotPose;
                            goto executePass;
                        }
                    }
                }
            }
            */
            if(theRobotPose.translation.x() > 1000.f && std::abs(theRobotPose.translation.y()) < 1500.f ){
                goto kick;
            }

            if(theRobotPose.translation.x() < -(float)theFieldDimensions.xPosPenaltyStrikerStartPosition){
                goto rinviocomechiellini;
            }


        }
        action
        {
            lookAtBall();
            bool waypoint_activated = false;  //waypoint:
            Vector2f obstPose;

            for(auto const &obstacle: theTeamPlayersModel.obstacles ){
                if(((theLibCodeRelease.norm(obstacle.center.x()-theRobotPose.translation.x(),
                     obstacle.center.y()-theRobotPose.translation.y())) < 1000.f ) &&
                     (obstacle.center.x() > theRobotPose.translation.x() + 10 ) &&
                     (obstacle.center.y() <= theRobotPose.translation.y() + 300 ||
                        obstacle.center.y() <= theRobotPose.translation.y() - 300) ){
                    waypoint_activated = true;
                    obstPose = Vector2f(obstacle.center.x(), obstacle.center.y());
                }
            }

            if (waypoint_activated && theBallModel.estimate.position.x() <= 200){
                (obstPose.y() > theRobotPose.translation.y()) ?
                            InWalkKick(WalkKickVariant(WalkKicks::right, Legs::left),
                                       Pose2f(theRobotPose.translation.angle(),
                                              theBallModel.estimate.position.x(),
                                              theBallModel.estimate.position.y() )) :
                            InWalkKick(WalkKickVariant(WalkKicks::left, Legs::right),
                                       Pose2f(theRobotPose.translation.angle(),
                                              theBallModel.estimate.position.x(),
                                              theBallModel.estimate.position.y() )) ;
            } else {
                //          SPQRWalkTo(Pose2f(1.f, 1.f, 1.f), Pose2f(theLibCodeRelease.angleToGoal, (float)theFieldDimensions.xPosOpponentGroundline, 0.f),
                //                     theLibCodeRelease.angleToGoal);

                WalkToTarget(Pose2f(1.0f, 1.0f, 1.0f),
                             Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position + Vector2f(200.0f, .0f)));
            }

            //Arms back

        }
    }

    state(kick)
    {
        transition
        {
//            std::cout<<"kick"<<std::endl;
            if(state_time > 50000 || (state_time > 10 && action_done)||
                theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut* 2 )
                goto start;
        }
        action
        {
            if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea && std::abs(theRobotPose.translation.y()) < 600){
                ChinaApproacher(true, Pose2f(theFieldDimensions.xPosOpponentGroundline+1000,theRobotPose.translation.y()));
            }else{
                ChinaApproacher(true, thePossiblePlan.betterTarget);
            }

            //std::cout<<"su bersaglio"<<std::endl;



        }
    }

    state(rinviocomechiellini)
    {
        transition
        {
//            std::cout<<"kick"<<std::endl;
            if(state_time > 50000 || (state_time > 10 && action_done)||
                theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut* 2)
                goto start;
        }
        action
        {

            if (theRobotPose.translation.x() < theFieldDimensions.xPosOwnDropInLine-300 &&
                    std::abs(theLibCodeRelease.angleToGoal) < Angle::fromDegrees(55.f) ){
                StrikerDemo();
            }else{
                ChinaApproacher(true, Pose2f((float)theFieldDimensions.xPosOpponentGroundline, 0.f));

            }
            //std::cout<<"su bersaglio"<<std::endl;


        }
    }
/*
    state(executePass){
        transition
        {
            //     std::cout<< "executePass" << std::endl;
            if(state_time > 30000 || (state_time > 10 && action_done))
                goto start;
        }
        action
        {
            if(justEnter_S == true){

                passTarget_S = passingMate_S;


                justEnter_S = false;
            }
            srand (time(NULL));
            lookAtBall();



            PassToTarget(passTarget_S);

        }
    }
*/

    state(searchForBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < 400)
                goto turnToBall;
        }
        action
        {
            //      theHeadControlMode = HeadControl::lookForward;
            //      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));

            if(theLibCodeRelease.timeSinceBallWasSeen < 300){
                lookAtGlobalBall();
            } else {
                SearchBallHead(1);
            }
        }
    }

#ifdef STATIC_STRATEGY
    state(passToJolly)
    {
        Vector2f mostAdvancedPlayer = theRobotPose.translation;
        transition
        {
            for(int i = 0; i < theTeamData.teammates.size(); i++){
                if(theTeamData.teammates.at(i).theRobotPose.translation.x() > mostAdvancedPlayer.x()){
                    mostAdvancedPlayer = theTeamData.teammates.at(i).theRobotPose.translation;
                }
            }

            if (theBallModel.estimate.position.norm() > 1200.f
//                    || mostAdvancedPlayer == theRobotPose.translation
                    || state_time > 12000.f)
                goto turnToBall;
        }
        action
        {

            /*if (mostAdvancedPlayer == theRobotPose.translation){
                InWalkKick(WalkKickVariant(WalkKicks::right, Legs::left),
                           Pose2f(theRobotPose.translation.angle(),
                                  theBallModel.estimate.position.x(),
                                  theBallModel.estimate.position.y() ));
            } else */{
//                ChinaApproacher(true,mostAdvancedPlayer);
                if (state_time > 4000)
                    PassToTarget(mostAdvancedPlayer); //TODO FIX

            }
        }
    }
#endif
}
