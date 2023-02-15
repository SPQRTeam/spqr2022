option(goalieFastDiveLeft)
{
  initial_state(goaliefastdiveLeft)
  {

    action
    {
        //lookAtBall();
      
        theMotionRequest.motion = MotionRequest::specialAction;
        theMotionRequest.specialActionRequest.mirror = false;
        theMotionRequest.specialActionRequest.specialAction = SpecialActionRequest::diveLeftFast;
  
    }
  }



}
