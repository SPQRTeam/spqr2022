
#define APPDISTX 190
#define YPOS 65
option(PenaltyApproacher, (const Pose2f&) target)
{

    float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y());
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
            if(std::abs(angle) <= Angle::fromDegrees(5.f) ){
                goto approach;
            }
        }
        action{
            lookAtBall();

            if(theBallModel.estimate.position.norm() < 250){
                if(angle >= 0){
                    WalkAtRelativeSpeed(Pose2f(0.5f, 0.f, -1.f));
                }else{
                    WalkAtRelativeSpeed(Pose2f(-0.5f, 0.f, 1.f));
                }
            }else{
                if(angle >= 0){
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

            if(std::abs(angle) <= Angle::fromDegrees(3.f + timeVar)){
                //std::cout<<"A"<<std::endl;
                if(theLibCodeRelease.between(theBallModel.estimate.position.x(), std::max((APPDISTX/2.f), 150.f), APPDISTX+10 + 3* timeVar)){
                    //std::cout<<"B"<<std::endl;
                    //std::cout<<"valore "<<std::abs(theBallModel.estimate.position.y())<<std::endl;
                    if(theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()), 35-timeVar, 50 + timeVar)){
                        //std::cout<<"C"<<std::endl;
                        goto kick;
                    }
                }
            }
        }
        action{
            lookAtBall();
            if(theBallModel.estimate.position.x() > APPDISTX+115){
                WalkToTarget(Pose2f(.5f, 0.8f, 0.8f),
                             Pose2f(angle,
                                    theBallModel.estimate.position.x() - APPDISTX,
                                    theBallModel.estimate.position.y() - ((theBallModel.estimate.position.y() > 0) ? YPOS : -YPOS)));
            }else{
                float timeVar = (state_time / 20000.f);
                WalkToTarget(Pose2f(0.2f, std::max((0.3f) - timeVar, 0.15f), std::max((0.3f) - timeVar, 0.15f)),
                             Pose2f(angle,
                                    theBallModel.estimate.position.x() - APPDISTX,
                                    theBallModel.estimate.position.y() - ((theBallModel.estimate.position.y() > 0) ? YPOS : -YPOS)));
            }
        }
    }

    state(kick){
        transition{
            if(state_time > 10000 || (state_time > 10 && action_done))
                goto targetState;
        }
        action{
            lookAtBall();
            Kicks("fastForwardKick");
        }
    }
    target_state(targetState){
        //std::cout<<"andato in target"<<std::endl;
    }

}
