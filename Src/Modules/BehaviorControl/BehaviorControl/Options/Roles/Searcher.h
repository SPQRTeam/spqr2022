#define SEARCH_BEHIND_OBSTACLES //if defined, makes the searcher 3 goes in his vision blindospots to search ball

option(Searcher, (const int) searcherNumber)
{
    float searcherXpose, searcherYpose;
    //actual element to search in theLibCodeRelease.placesToSearchBall
    uint placeToSearch = 0;
    if (searcherNumber == 4) {
        // searcherXpose = 2800; searcherYpose = +1500;  // gloabalBall
#ifdef SEARCH_BEHIND_OBSTACLES
         if(theLibCodeRelease.placesToSearchBall.size() > 0 && placeToSearch <= theLibCodeRelease.placesToSearchBall.size() - 1){
            //if I am close enough to the search position, go to next
            if((Vector2f(theRobotPose.translation.x(), theRobotPose.translation.y()) - theLibCodeRelease.placesToSearchBall.at(placeToSearch)).norm() <= 400.f)
                placeToSearch++;
            searcherXpose = theLibCodeRelease.placesToSearchBall.at(placeToSearch).x();
            searcherYpose = theLibCodeRelease.placesToSearchBall.at(placeToSearch).y();
            }
        else{
#endif
            placeToSearch = 0;
            //SET the robot position between the guard positions
            searcherXpose = SPQR::GUARD_X_POSE; searcherYpose = SPQR::GUARD_Y_POSE;
#ifdef SEARCH_BEHIND_OBSTACLES
        }
#endif
    } else if (searcherNumber == 3){
        // searcherXpose = 2800; searcherYpose = -1500;    // cambio y in base al 3
        // searcherXpose = 2800;
        //if(theTeamBallModel.position.y() > 0)

        if(theTeamBallModel.timeWhenLastSeen < 20000.f){
            if( theRobotPose.translation.x() < 200){  // if robot in my field
                if( theTeamBallModel.position.y() > 0){
                    searcherYpose = ( theTeamBallModel.position.y() - 1500 );
                    if(theTeamBallModel.position.x() < -3000)
                    searcherXpose = (int)-(theFieldDimensions.xPosPenaltyStrikerStartPosition+400);
                    else
                    searcherXpose = (int)theTeamBallModel.position.x();
                }
                else{
                searcherYpose = (int)(theTeamBallModel.position.y() + 1500);
                    if( theTeamBallModel.position.x() < -3000.f ) searcherXpose = -(theFieldDimensions.xPosPenaltyStrikerStartPosition+400);
                    else searcherXpose = theTeamBallModel.position.x();
                }

            }
            else{
                if(theTeamBallModel.position.y() > 0)
                searcherYpose = (int)(theTeamBallModel.position.y() - 1000.f);
                else
                searcherYpose = (int)(theTeamBallModel.position.y() + 1000.f);
                searcherXpose = (int)(theTeamBallModel.position.x() - 500.f);
            }
        }
        else
        {
            if(theTeamBallModel.position.y() > 0)
                searcherYpose = 800.f;
            else
                searcherYpose = -800.f;

            searcherXpose = 1000.f;
        }


    }
//    std::cout<< "searcher n: " << searcherNumber << " pose: " << searcherXpose << ", " <<searcherYpose << std::endl;

    initial_state(start)
    {
        transition
        {   
            if(state_time > 1500.f){
                goto start2;
            }
            
        }
        action
        {
            lookAtGlobalBall();
            
            Stand();
        }
    }
    state(start2){
        transition{
            if(state_time > 2000.f){
                goto turnToCentroid;
            }
        }action{
            lookLeftAndRight();
            if(theRobotPose.translation.x() > 2000){
                WalkAtRelativeSpeed(Pose2f(0.f,-0.4f,0.f));
            }else{
                Stand();
            }
            
        }
    }
    common_transition
    {
        theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
        theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = ArmKeyFrameRequest::back;
        theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
        theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = ArmKeyFrameRequest::back;

    }

    state(turnToCentroid)
    {
        transition
        {
            if(std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(searcherXpose,
                                                        searcherYpose))) < (20.f))
               goto walkToCentroid;
        }
        action
        {
            LookForward();
            if((theLibCodeRelease.angleToTarget(searcherXpose, searcherYpose)) > 0)
                WalkAtRelativeSpeed(Pose2f(50.f, 0.0001f,0.0001f));
            else
                WalkAtRelativeSpeed(Pose2f(-50.f, 0.0001f,0.0001f));
        }
    }

    state(walkToCentroid)
    {
        transition
        {
            // if(std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(searcherXpose,
            //                                             searcherYpose))) > (20.f))
            //     goto turnToCentroid;

            if( std::abs(theRobotPose.translation.x() - searcherXpose) < 250 &&
                std::abs(theRobotPose.translation.y() - searcherYpose) < 250 )
                goto lookAround;

        }
        action
        {
            lookLeftAndRight();
            // WalkAtRelativeSpeed(Pose2f(0.0001f, 50.f,0.0001f));
            WalkToTargetPathPlanner(Pose2f(0.8f,0.8f,0.8f),Pose2f(searcherXpose,searcherYpose));
        }
    }

    state(lookAround)
    {
        transition
        {
           if( std::abs(theRobotPose.translation.x() - searcherXpose) > 500 ||
               std::abs(theRobotPose.translation.y() - searcherYpose) > 500  && state_time > 18000 )
               goto walkToCentroid;

            // if(state_time > 9000)
            //     goto walkToCentroid;

            // if(theLibCodeRelease.timeSinceBallWasSeen < 300)
            //     goto walkToCentroid;

        }
        action
        {
            // lookLeftAndRight();
            // WalkAtRelativeSpeed(Pose2f(50.f, 0.0001f,0.0001f));
            (theBallModel.lastPerception.y() > 0) ?
                SearchBallHead(1) : SearchBallHead(-1);   // head and body rotation
        }
    }

//     state(walkAnother)
//     {
//         int searcherX;
//         int searcherY;
//         transition
//         {
//             if(searcherNumber == 3)
//             {
//                 if(searcherX <= -2000)
//                     searcherX = -500;

//                 else if (searcherX  >-2000  &&  searcherX <= 0)
//                     searcherX = 1200;
//                 else if (searcherX  >0  &&  searcherX <= 2500)
//                     searcherX = 3200;
//                 else
//                     searcherX = 3800;

//                 searcherY = theTeamBallModel.position.y()/std::abs(theTeamBallModel.position.y()) *1500;

//             }
//             else
//             {
//                 searcherX = searcherXpose - 2000;
//                 searcherY = searcherYpose;
//             }
// //            if(std::abs(theLibCodeRelease.radiansToDegree(theLibCodeRelease.angleToTarget(searcherX,
// //                                                        searcherY))) > (20.f))
// //                goto turnToCentroidAnother;

//             if( std::abs(theRobotPose.translation.x() - searcherX) < 200 &&
//                 std::abs(theRobotPose.translation.y() - searcherY) < 200 )
//                 goto lookAround;

//         }
//         action
//         {
//             lookLeftAndRight();
//             SPQRWalkTo(Pose2f(.9f, .9f, .9f),
//                        Pose2f(searcherX, searcherY), theLibCodeRelease.angleToTarget(searcherX, searcherY));

//         }
//     }
}
