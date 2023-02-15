option(Stopper) {
    initial_state(start) {
        transition {
            Vector2f velocity = theBallModel.estimate.velocity;
            Vector2f position = theBallModel.estimate.position;
            if(theGameInfo.state == STATE_PLAYING  &&
                    velocity.x() < 0 &&
                    velocity.norm()/position.norm() > 0.8){

                double teta = atan(velocity.y()/velocity.x());
                float l = position.x()*tan(teta);
                float lato = position.y()-l;
                if(lato<=200 && lato >= -200)
                    goto stopBall;
            }
        }
        action {
            Stand();
            if(theLibCodeRelease.timeSinceBallWasSeen < 2000) {
                lookAtBall();
            } else if(theLibCodeRelease.timeSinceBallWasSeen < 5000) {
                lookAtGlobalBall();
            } else {
                lookLeftAndRight();
            }
        }
    }

    state(stopBall)
    {
        transition
        {
            if (action_done && state_time > 10000.f)
                goto start;
        }
        action
        {
            lookAtBall();
            StopBall();
        }
    }
}
