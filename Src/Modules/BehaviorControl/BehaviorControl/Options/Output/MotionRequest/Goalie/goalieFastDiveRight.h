option(goalieFastDiveRight)
{
  initial_state(goaliefastdiveRight)
  {

    action
    {
        //lookAtBall();
      
        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = true;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::diveLeftFast;
  
    }
  }



}
