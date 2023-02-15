option(TrialWalk)
{
    initial_state(start)
    {
        transition{
            if(state_time > 4000){
               goto side; 
            }
        }action{
            WalkAtRelativeSpeed(Pose2f(0.f,1.f,0.f));
        }
    }
    state(side){
        transition{
            if(state_time > 4000){
                goto back;
            }
        }action{
            WalkAtRelativeSpeed(Pose2f(0.f,0.f,1.f));

        }
    }
    state(back){
        transition{
            if(state_time > 4000){
                goto turn;
            }
        }action{
            WalkAtRelativeSpeed(Pose2f(0.f,-1.f,0.f));

        }
    }
    state(turn){
        transition{
            if(state_time > 4000){
                goto runup;
            }
        }action{
            WalkAtRelativeSpeed(Pose2f(1.f,0.f,0.f));
        }
    }
    state(runup){
        transition{
            if(state_time > 4000){
                goto stand0;
            }
        }action{
            RunUpWalkToTarget(Pose2f(1.f,1.f,1.f), Pose2f(theBallModel.estimate.position));
        }
    }
    state(stand0){
        transition{
            if(state_time > 4000){
                goto kickLeft;
            }
        }action{
            Stand();
        }
    }
    state(kickLeft){
        transition{
            if(state_time > 6000){
                goto stand;
            }
        }action{
            InWalkKick(WalkKickVariant(WalkKicks::left, Legs::right), Pose2f(25_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

        }
    }
    state(stand){
        transition{
            if(state_time > 4000){
                goto kickRight;
            }
        }action{
            Stand();
        }
    }
    state(kickRight){
        transition{
            if(state_time > 6000){
                goto stand2;
            }
        }action{
            InWalkKick(WalkKickVariant(WalkKicks::right, Legs::left), Pose2f(25_deg, theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));

        }
    }
    state(stand2){
        transition{
            
        }action{
            Stand();
        }
    }

}

