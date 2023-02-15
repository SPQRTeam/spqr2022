option(ChinaApproacher, (bool) toKick, (const Pose2f&) target)
{

    /* float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y()); */
    Pose2f globBall = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
    float ballTargetDist = theLibCodeRelease.norm(std::abs(globBall.translation.x()-target.translation.x()), std::abs(globBall.translation.y()-target.translation.y()));
    float wttAngle = std::asin( ( target.translation.y()-globBall.translation.y() )/ballTargetDist );
    initial_state(start)
    {
        transition
        {
          if(theLibCodeRelease.between(theRobotPose.translation.x(), globBall.translation.x()-400,globBall.translation.x()-5) && theLibCodeRelease.between(theRobotPose.translation.y(), globBall.translation.y()-400,globBall.translation.y()+400)){
              goto approacher;
          }
            else{
            goto walkPathPlanner;
          }

        }
        action
        {
            lookAtBall();
            Stand();
        }
    }

    common_transition
    {
        /*
        if(toKick == true){
            if(theBallModel.estimate.position.norm() > 800){
                goto targetState;
            }
        }else{
            if(theBallModel.estimate.position.norm() > 10500){
                goto targetState;
            }
        }*/

    }

    state(walkPathPlanner){
      transition
        {
        if(theLibCodeRelease.distance(Pose2f( theRobotPose.translation),
                ( globBall - Pose2f( std::abs( 300* std::cos( wttAngle ) ), 120* std::sin( wttAngle ) ) ) ) < 100 &&
                 theRobotPose.translation.x() < globBall.translation.x() ){

                goto approacher;
            }
            if(theLibCodeRelease.distance(Pose2f( theRobotPose.translation),
                ( globBall - Pose2f( std::abs( 300* std::cos( wttAngle ) ), 120* std::sin( wttAngle ) ) ) ) < 300 && state_time > 10000) {
                    goto approacher;
            }
      }
        action
        {
        lookAtBall();
        if(theRobotPose.translation.y()>= 0){
          WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(-wttAngle, globBall.translation.x(),globBall.translation.y()) - Pose2f(0, std::abs(300* std::cos(wttAngle)), 120* std::sin(wttAngle)));
        }
            else{
          WalkToTargetPathPlanner(Pose2f(1.f,1.f,1.f), Pose2f(wttAngle, globBall.translation.x(),globBall.translation.y()) - Pose2f(0, std::abs(300* std::cos(wttAngle)), 120* std::sin(wttAngle)));
        }
      }
    }
    state(approacher){
      transition
        {
            /*
        if(state_time > 15000 || action_done){
          goto targetState;
        }
            if( action_aborted){
                goto walkPathPlanner;
            }*/
            if(theBallModel.estimate.position.norm()>600)
                goto walkPathPlanner;
      }
        action
        {
            lookAtBall();
            Approacher(toKick, target);
        }
    }

    /* target_state(targetState){ */
    /*     transition{ */
    /*         if(state_time > 2000){ */
    /*             //std::cout<<"state time chinaApproacher target state  "<<state_time<<std::endl; */
    /*             goto start; */
    /*         } */
    /*     }action{ */
    /*     } */
    /* } */

    /* aborted_state(abort) */
    /* { */
    /*     //std::cout<<"state time chinaApproacher aborted state  "<<state_time<<std::endl; */
    /* } */


}
