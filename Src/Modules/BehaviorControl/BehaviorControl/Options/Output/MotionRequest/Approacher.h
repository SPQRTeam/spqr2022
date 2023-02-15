option(Approacher, (bool) toKick, (const Pose2f&) target)
{
    std::string kickName;
  kickName = "strongKick";

    if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation), target) < 4500.f){

        kickName = "strongKick";

    }
    if(theLibCodeRelease.distance(Pose2f(theRobotPose.translation), target) < 2800.f){
        kickName = "pass_2";
    }
    if(target.translation.x() > theFieldDimensions.xPosOpponentGroundline - 100){
        if(std::abs(target.translation.y()) < theFieldDimensions.yPosLeftGoal){
            

            kickName = "strongKick";
        }
    }


  //distances from ball
  float d1,d2,d3;
  //distancese for between ball
  float bt1,bt2,bt3;
  //questi sembrano ok, forse da cambiare qualcosa sui bt per la precisione del tiro
  Pose2f myTarget = target;
  if(kickName == "strongKick"){
    d1 = 260; //x displacement in transition
    d2 = 260; //x displacement in if
    d3 = 265; // x displacement in action
    bt1 = 65; //y transition
    bt2 = 65; // y large
    bt3 = 62; // y strict
    float distX = std::abs(target.translation.x() - theRobotPose.translation.x());
    float distY = std::abs(target.translation.y() - theRobotPose.translation.y());
    float ratio = distY/distX;
    if(target.translation.y() > theRobotPose.translation.y()){
      if(target.translation.x() > theRobotPose.translation.x()){
          Pose2f myTarget =  Pose2f(theRobotPose.translation.x() + 1000.f,theRobotPose.translation.y() + 1000 * ratio );
      }else{
          Pose2f myTarget =  Pose2f(theRobotPose.translation.x() - 1000.f,theRobotPose.translation.y() + 1000 * ratio );
      }
      
    }else{
      if(target.translation.x() > theRobotPose.translation.x()){
          Pose2f myTarget =  Pose2f(theRobotPose.translation.x() + 1000.f,theRobotPose.translation.y() - 1000 * ratio );
      }else{
          Pose2f myTarget =  Pose2f(theRobotPose.translation.x() - 1000.f,theRobotPose.translation.y() - 1000 * ratio );
      }
    }
    
  }else if(kickName == "lobKick"){
    d1 = 160; //x displacement in transition
    d2 = 160; //x displacement in if
    d3 = 160; // x displacement in action
    bt1 = 50; //y transition
    bt2 = 50; // y large
    bt3 = 47; // y strict
    Pose2f myTarget = target;
  }else{
    d1 = 170; //x displacement in transition
    d2 = 170; //x displacement in if
    d3 = 165; // x displacement in action
    bt1 = 45; //y transition
    bt2 = 45; // y large
    bt3 = 43; // y strict
    Pose2f myTarget = target;
    }


    float angle = theLibCodeRelease.angleToTarget(myTarget.translation.x(), myTarget.translation.y());

    initial_state(start)
    {

            transition
            {

                goto turnToBall;
            }
            action
            {
                Stand();

            }
    }

    common_transition
    {

    }

    state(turnToBall){
        transition{
            if(state_time > 10000 || (state_time > 10 && action_done))
                goto moveAroundBall;

        }
        action{
            Pose2f ballPose = theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());
            Turn(ballPose);
        }

    }

    state(moveAroundBall){
        transition{
            if(!toKick){
                if(std::abs(angle) < Angle::fromDegrees(20.f) ){
                    goto targetState;
                }
            }else{
                if(theRobotPose.translation.x() > 4100.f){
                    if(std::abs(angle) <= Angle::fromDegrees(6.f) ){
                        goto approach;
                    }
                }
                //std::cout<<"angle "<<angle<<std::endl;
                if(std::abs(angle) <= Angle::fromDegrees(6.f) ){
                    goto approach;
                }
            }//else tokick



        }
        action{//action of MoveAroundBall

            if(!toKick){
                if(std::abs(angle) > Angle::fromDegrees(40.f)){
                    if(angle >= 0){
                        WalkAtRelativeSpeed(Pose2f(0.5f, 0.1f, -1.f));
                    }else{
                        WalkAtRelativeSpeed(Pose2f(-0.5f, 0.1f, 1.f));
                    }
                }else{
                    if(angle >= 0){
                        WalkAtRelativeSpeed(Pose2f(0.5f, 0.1f, -0.6f));
                    }else{
                        WalkAtRelativeSpeed(Pose2f(-0.5f, 0.1f, 0.6f));
                    }
                }


            }else{
                if(std::abs(angle) > Angle::fromDegrees(40.f)){
                    if(angle >= 0){
                        WalkAtRelativeSpeed(Pose2f(0.55f, 0.1f, -0.9f));
                    }else{
                        WalkAtRelativeSpeed(Pose2f(-0.55f, 0.1f, 0.9f));
                    }
                }else{
                    if(angle >= 0){
                        WalkAtRelativeSpeed(Pose2f(0.5f, 0.1f, -0.6f));
                    }else{
                        WalkAtRelativeSpeed(Pose2f(-0.5f, 0.1f, 0.6f));
                    }
                }


            }



        }

    }//MoveAroundBall



    state(approach){

        transition{
          // parametrize the approach w.r.t to the position
          // the more the distance from the center field the less the threshold
          float distanceNormalization = 2.1+std::abs(theRobotPose.translation.x())/theFieldDimensions.xPosOpponentGroundline;
          float timeVar = (state_time / 1000)*distanceNormalization;
          //250 strong
          if(theLibCodeRelease.between(theBallModel.estimate.position.x(),0.f,d1 + timeVar)){
            //std::cout<<"B"<<std::endl;
            if(theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()), bt1 - timeVar,bt1 + timeVar) &&
               !theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()), 20 ,25 )){
              //std::cout<<"C"<<std::endl;
              goto kick;
            }
          }
        }
        action{//action of approach
            if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea){
                WalkAtRelativeSpeed(Pose2f(0.f,1.f,0.f));
            }else{
                if(theBallModel.estimate.position.x() > d2){
                    WalkToTarget(Pose2f(.5f, .8f, .8f),
                                Pose2f(0,
                                        theBallModel.estimate.position.x() - d2,
                                        theBallModel.estimate.position.y() - ((theBallModel.estimate.position.y() > 0) ? bt2 : -bt2)));

                }else{
                    /* float timeVar = (state_time / 20000.f); */
                    WalkToTarget(Pose2f(.5f, 0.4f,0.4f),
                                Pose2f(0,
                                        theBallModel.estimate.position.x() - d3,
                                        theBallModel.estimate.position.y() - ((theBallModel.estimate.position.y() > 0) ? bt3 : -bt3)));

                }
            }
        }


    }

    state(kick){
        transition{
            if(state_time > 5000 /*|| (state_time > 10 && action_done) TODO EMANUELE REINSERT*/)
                goto start;//goto targetState;

        }
        action{

            if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea && std::abs(theRobotPose.translation.y())< 600.f )){
                WalkAtRelativeSpeed(Pose2f(0.f,1.f,0.f));
            }else{
                if(state_time < 300){
                    Stand();
                }else{
                    if(kickName == "pass_2"){
                        if(theBallModel.estimate.position.y() < 0){
                            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::right), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
                        }else{
                            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(0_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
                        }
                    }
                    else {
                      Kicks(kickName);
                    }
                }

            }
        }
    }
    target_state(targetState){
        action{
            Stand();
        }
    }

}
