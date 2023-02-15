/*
mode 0 = kick
mode 1 = walk with ball
mode 2 = inWalkKick
*/
option(Approacher2020, (int) mode, (const Pose2f&) target)
{
    Pose2f globball = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    if(mode < 0 || mode > 2){
        mode = 1;
    }
    initial_state(start)
    {

            transition
            {   
                if(target.translation.x() > globball.translation.x()){
                    goto reachBallFront;
                }else{
                    goto reachBallBack;
                }
                
            }
            action
            {
                Stand();

            }
    }

    state(reachBallFront){
        transition{
            if(theRobotPose.translation.x() < globball.translation.x() - 50.f){
                if(theBallModel.estimate.position.norm() < 400.f){
                    if(std::abs(theRobotPose.translation.y() - globball.translation.y()) < 50.f){
                        if(theRobotPose.translation.y() > globball.translation.y()){
                            goto gotoBallRight;
                        }else{
                            goto gotoBallLeft;
                        }
                    }
                    
                }
            }
        }action{
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(0, globball.translation.x() - 200.f,globball.translation.y()) );
            if(theBallModel.estimate.position.norm() > 500.f){
                lookAtBall();
            }else{
                lookAtBall();
            }
            
        }
    }

    state(reachBallBack){
        transition{
            if(theRobotPose.translation.x() > globball.translation.x() + 50.f){
                if(std::abs(theBallModel.estimate.position.y()) < 150.f){
                    
                    if(theBallModel.estimate.position.norm() < 400.f){
                        
                        if(theRobotPose.translation.y() > globball.translation.y()){
                            goto gotoBallLeft;
                        }else{
                            goto gotoBallRight;
                        }
                    }
                }
                
            }
        }action{
            WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(3.14, globball.translation.x() + 200.f,globball.translation.y()) );
            if(theBallModel.estimate.position.norm() > 500.f){
                lookAtBall();
            }else{
                lookAtBall();
            }
        }
    }

    

    state(gotoBallLeft){
        transition{
            if(mode == 0){

                if(theBallModel.estimate.position.x() < 225.f){

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 55.f, 70.f ) ){

                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(6.f) ){
                            goto kick;
                        }// if angle

                    }// if y
                    
                }// if x
                
            }else if(mode == 1){
                if(theBallModel.estimate.position.x() < 180.f){

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 30.f, 90.f ) ){

                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(7.f) ){
                            goto ballAhead;
                        }// if angle

                    }// if y
                    
                }// if x
            }else if(mode == 2 ){
                if(theBallModel.estimate.position.x() < 180.f){
                    std::cout << "left x control" << std::endl;

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 40.f, 80.f ) ){
                        std::cout << "left y control" << std::endl;
                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(7.f) ){
                            std::cout << "left th control" << std::endl;
                            goto InWalkKick;
                        }// if angle

                    }// if y
                    
                }// if x

            }//if else mode
        }
        action{
            if(mode == 0){
                WalkAtAbsoluteSpeed(Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(),target.translation.y()) * 5,
                theBallModel.estimate.position.x() - 225.f,theBallModel.estimate.position.y() - 62.f) );
            
            }else{
                WalkAtAbsoluteSpeed(Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(),target.translation.y()) * 5,
                theBallModel.estimate.position.x() - 150.f,theBallModel.estimate.position.y() - 60.f) );
            
            }
            lookAtBall();
        }
    }// gotoBallLeft


    state(gotoBallRight){
        transition{
            if(mode == 0){

                if(theBallModel.estimate.position.x() < 220.f){

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 55.f, 70.f ) ){

                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(6.f) ){
                            goto kick;
                        }// if angle

                    }// if y
                    
                }// if x
                
            }else if(mode == 1){
                if(theBallModel.estimate.position.x() < 180.f){

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 30.f, 90.f ) ){

                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(7.f) ){
                            goto ballAhead;
                        }// if angle

                    }// if y
                    
                }// if x
            }else if(mode == 2 ){
                if(theBallModel.estimate.position.x() < 180.f){
                    std::cout << "left x control" << std::endl;

                    if( theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()) , 40.f, 80.f ) ){
                        std::cout << "left y control" << std::endl;
                        if(std::abs(theLibCodeRelease.angleToTarget(target.translation.x(),
                        target.translation.y())) < Angle::fromDegrees(7.f) ){
                            std::cout << "left th control" << std::endl;
                            goto InWalkKick;
                        }// if angle

                    }// if y
                    
                }// if x
                
            }//if else mode
        }
        action{
            if(mode == 0){
                WalkAtAbsoluteSpeed(Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(),target.translation.y()) * 5,
                theBallModel.estimate.position.x() - 210.f,theBallModel.estimate.position.y() + 65.f) );
            
            }else{
                WalkAtAbsoluteSpeed(Pose2f(theLibCodeRelease.angleToTarget(target.translation.x(),target.translation.y()) * 5,
                theBallModel.estimate.position.x() - 150.f,theBallModel.estimate.position.y() + 60.f) );
            
            }
            lookAtBall();
        }
    }//gotoBallRight

    


    state(ballAhead){
       transition{
           if(state_time > 2000 || action_done || theBallModel.estimate.position.norm() > 400.f || theBallModel.estimate.position.x() < -10){
               goto start;
           }
       }action{
           lookAtBall();
           if(theBallModel.estimate.position.y() <= 0){
               InWalkKick(WalkKickVariant(WalkKicks::forward_little, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
       
           }else{
               InWalkKick(WalkKickVariant(WalkKicks::forward_little, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
       
           }
        }
   }

   state(kick){
       transition{
           if(state_time > 2000 || action_done || theBallModel.estimate.position.norm() > 400.f || theBallModel.estimate.position.x() < -10){
               goto start;
           }
       }action{
           Kicks("strongKick");
       }
   }

   state(InWalkKick){
       transition{
           if(state_time > 2000 || action_done || theBallModel.estimate.position.norm() > 400.f || theBallModel.estimate.position.x() < -10){
               goto start;
           }
       }action{
            lookAtBall();
            if(theBallModel.estimate.position.y() <= 0){
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

            }else{
                InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
    
            }
           
           
        }
   }
}
