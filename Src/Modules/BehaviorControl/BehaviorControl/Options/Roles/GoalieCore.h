option(GoalieCore)
{
	initial_state(start)
    {
        transition
        {
#ifdef PENALTY_STRIKER_GOALIE            
            goto goaliePose;
#endif
            goto loop;               
        }
        action
        {
            Stand();
        }
    }

    state(goaliePose)
    {
        transition
        {
            if( state_time>1000)
                goto loop;
        }
        action
        {
            WalkAtRelativeSpeed(Pose2f(0.f, 1.f,0.f));
        }
    }

    state(loop)
    {  
        transition{
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;

#ifdef PENALTY_STRIKER_GOALIE  
            if(theGameInfo.state == STATE_PLAYING  && velocity.x()<0 && velocity.norm()/position.norm()>0.7){ 
#endif     
                double teta = atan(velocity.y()/velocity.x());
                float l = position.x()* (float) tan(teta);
                float lato = position.y()-l;

                if(lato<=200.f && lato >= -200.f)
                    goto stopBall;
                else if(lato>200.f && lato < 1500.f)
                    goto goalieDiveLeft;
                else if(lato<-200.f && lato>-1500.f)
                    goto goalieDiveRight;
#ifdef PENALTY_STRIKER_GOALIE  
            }
#endif     
        }
        action
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000){
                Stand();
                lookAtBall();
            }
#ifndef PENALTY_STRIKER_GOALIE       
            else if(theLibCodeRelease.timeSinceBallWasSeen < 5000){
                Stand();
                lookAtGlobalBall();
            }
#endif              
            else{
                Stand();
                lookLeftAndRight();
            }
        }
    }
	
	state(goalieDiveLeft)
    {
        action
        {
            lookAtBall();
            goalieFastDiveLeft();
        }
    }

    state(goalieDiveRight)
    {
        action
        {
            lookAtBall();
            goalieFastDiveRight();
        }
    }

    state(stopBall)
    {
        action
        {
            lookAtBall();
            StopBall();
        }
    }
}