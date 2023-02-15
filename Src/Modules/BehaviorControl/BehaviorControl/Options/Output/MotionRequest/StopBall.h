
option(StopBall)
{
  initial_state(stopBall)
  {
    action
    {
        theHeadControlMode = HeadControl::lookAtBall;

        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = true;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::stopBall;
    }
  }
}


