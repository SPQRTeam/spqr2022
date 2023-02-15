#define PRE_APPDISTX 400
#define PRE_YPOS 60
option(PenaltyPreApproacher)
{

    //    common_transition{
    //        std::cout<< "angle: " << angle*57.f<< std::endl;
    //    }

    initial_state(start)
    {
        transition
        {
            goto turnToBall;
        }
        action
        {
            Stand();
            lookAtBall();
        }
    }

    common_transition
    {
        if(theBallModel.estimate.position.norm() > 600){
            goto targetState;
        }
    }

    state(turnToBall){
        transition{
            if(state_time > 10000 || (state_time > 10 && action_done))
                goto moveAroundBall;
        }
        action{

            lookAtBall();
            Pose2f ballPose = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
            Turn(ballPose);
        }
    }

    state(moveAroundBall){
        transition{

            //std::cout<<"angle "<<angle<<std::endl;
            if(std::abs(theLibCodeRelease.angleToGoal) <= Angle::fromDegrees(5.f) ){
                goto approach;
            }
        }
        action{
            lookAtBall();

            if(theBallModel.estimate.position.norm() < 250){
                if(theLibCodeRelease.angleToGoal >= 0){
                    WalkAtRelativeSpeed(Pose2f(0.5f, 0.f, -1.f));
                }else{
                    WalkAtRelativeSpeed(Pose2f(-0.5f, 0.f, 1.f));
                }
            }else{
                if(theLibCodeRelease.angleToGoal >= 0){
                    WalkAtRelativeSpeed(Pose2f(0.5f, 0.1f, -1.f));
                }else{
                    WalkAtRelativeSpeed(Pose2f(-0.5f, 0.1f, 1.f));
                }
            }
        }
    }

    state(approach){
        transition{
            float timeVar = (state_time / 1000);

            if(theLibCodeRelease.between(theBallModel.estimate.position.x(), std::max((PRE_APPDISTX/2.f), 150.f), PRE_APPDISTX+10 + 3* timeVar)){

                goto targetState;
            }
        }
        action{
            lookAtBall();

                WalkToTarget(Pose2f(.5f, 1.f, 1.f),
                             Pose2f(theLibCodeRelease.angleToGoal,
                                    theBallModel.estimate.position.x() - PRE_APPDISTX,
                                    theBallModel.estimate.position.y() - ((theBallModel.estimate.position.y() > 0) ? PRE_YPOS : -YPOS)));

        }
    }


    target_state(targetState){
        //std::cout<<"andato in target"<<std::endl;
    }

}
