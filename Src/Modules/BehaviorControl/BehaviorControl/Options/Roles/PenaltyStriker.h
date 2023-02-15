#define APPROACH_THE_BALL_THRESHOLD 300.f
option(PenaltyStriker)
{

    initial_state(start)
    {
        transition
        {
            if(state_time > 5000)
                goto walkToBall;
        }
        action
        {
            lookLeftAndRight();
            Stand();
        }
    }

    state(walkToBall){
        transition{
            if (theBallModel.estimate.position.norm() < APPROACH_THE_BALL_THRESHOLD){
                goto approach;
            }
        }
        action{
            lookAtBall();
            Pose2f ballGlob =  theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x() , theBallModel.estimate.position.y() ) ;
            SPQRWalkTo(Pose2f(0.7f, 0.7f, 0.7f),
                       Pose2f(ballGlob.translation.x(), ballGlob.translation.y()  ), theLibCodeRelease.angleToGoal);
            
        }
    }

//    state(preapproach){
//        transition{
//            if(state_time > 10000 || action_done){
//                goto approach;
//            }
//        }
//        action{
//            PenaltyPreApproacher();
//        }

//    }

//    state(secondLook){
//        transition{
//            if(state_time > 3000){
//                goto approach;
//            }
//        }action{
//            lookLeftAndRight();
//        }
//    }

    state(approach){
        transition{
            if(state_time > 24000)
                goto start;
        }action{
            float opponentY;
            bool thereIsGoalie = false;
            int i;
            for(i = 0; i < theTeamPlayersModel.obstacles.size(); i++){
                if(theTeamPlayersModel.obstacles.at(i).type == Obstacle::opponent){
                    opponentY = theTeamPlayersModel.obstacles.at(i).center.y();
                    thereIsGoalie = true;
                    break;
                }
            }
            if(thereIsGoalie){
                
                
                /*if(opponentY >= 0){
                    std::cout<<"oppy >0"<<std::endl;
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, -700.f));
                }else{
                    std::cout<<"oppy <0"<<std::endl;
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, 700.f));

                }*/

                Pose2f kickTarget;
                if(thePossiblePlan.betterTarget.translation.y()>=0){
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, 490.f));
                }else{
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, -490.f));
                }
                
            }else{
                //std::cout <<"opp < 0 " << std::endl;
                srand (time(NULL));
                if(((rand()*100 % 100)) >= 50 ){
                  //  std::cout<<"caso1"<<std::endl;
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, -490.f));

                }else{
                    //std::cout<<"caso2"<<std::endl;
                    PenaltyApproacher(Pose2f((float)theFieldDimensions.xPosOpponentGroundline, 490.f));

                }  
            }
        }
    }
}
