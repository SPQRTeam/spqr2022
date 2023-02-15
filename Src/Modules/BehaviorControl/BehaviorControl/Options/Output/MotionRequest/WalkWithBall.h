//This option takes as input the absolute target and the speed of the robot

option(WalkWithBall, (const Pose2f&) speed, (const Pose2f&) target)
{

    float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y());
    float wttAngle = -theLibCodeRelease.angleToTarget(target.translation.x(),target.translation.y());
    initial_state(start)
    {
        transition
        {

            goto approachTheBall;


        }
        action
        {
            Stand();
            lookAtBall();
        }
    }

    state(approachTheBall){
        Pose2f globBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());

        transition
        {
            if(theLibCodeRelease.between(target.translation.x(), theRobotPose.translation.x() - 200, theRobotPose.translation.x() + 200 ) ){
                if(target.translation.y() > theRobotPose.translation.y() ){
                    if( theRobotPose.translation.x() < globBall.translation.x() ){
                        if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation),
                        Pose2f(0, globBall.translation.x() , globBall.translation.y() - 100  ) ) < 100){
                            goto turnToTarget;
                        }
                    }
                }else{
                    if( theRobotPose.translation.x() < globBall.translation.x() ){
                        if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation),
                        Pose2f(0, globBall.translation.x() , globBall.translation.y() + 100  ) ) < 100){
                            goto turnToTarget;
                        }
                    }
                }
                
            }else{
                if( theRobotPose.translation.x() < globBall.translation.x() ){
                    if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation),
                    Pose2f(0, globBall.translation.x() - 150 , globBall.translation.y() - std::sin(angle)*80) ) < 200){
                        goto turnToTarget;
                    }
                }
            }
            
        }
        action
        {
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(0, globBall.translation.x() - 150 , globBall.translation.y() - std::sin(angle)*80  ));
            
        }

    }

    state(turnToTarget){
        transition{
            if(std::abs(angle) <= Angle::fromDegrees(8.f) ){
                std::cout << "angle turn = " << std::abs(angle) << std::endl;
                goto walk;

            }

        }action{
            Turn(target);

        }

    }

    state(walk){
        transition{
            if(theBallModel.estimate.position.norm() > 300 || theBallModel.estimate.position.x() < 0 || std::abs(theBallModel.estimate.position.y()) > 200 ){
                goto approachTheBall;
            }
        }action{
            if(theRobotPose.translation.x() > 3800.f){
                if(theBallModel.estimate.position.y() <= 0){
                        
                        InWalkKick(WalkKickVariant(WalkKicks::forward_little, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

                }else{
                        InWalkKick(WalkKickVariant(WalkKicks::forward_little, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

                }
            }else{
                if(theBallModel.estimate.position.y() <= 0){
                        
                        InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

                }else{
                        InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

                }
            }
            
        }

    }


}
