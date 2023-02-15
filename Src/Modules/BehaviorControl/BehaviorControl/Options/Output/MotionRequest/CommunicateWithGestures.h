option(CommunicateWithGestures, (const std::string) gesture)
{
    initial_state(gesture)
    {
        transition{
            if(gesture == "oneArm")
                goto oneArm;
            else if(gesture == "twoArms")
                goto twoArms;
            else if(gesture == "armOnTheSide")
                goto armOnTheSide;
            else
                throw std::invalid_argument( "received wrong kick ID" );
        }
    }

    state(oneArm)
    {
        // transition{
        //     if(state_time >= 1500.f)
        //         goto target_state;
        // }
        action
        {   
            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::oneArm;
        }
    }


    state(twoArms)
    {
        // transition{
        //     if(state_time >= 1500.f)
        //         goto target_state;
        // }
        action
        {   
            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::twoArms;
        }
    }
    
    state(armOnTheSide)
    {
        // transition{
        //     if(state_time >= 1500.f)
        //         goto target_state;
        // }
        action
        {   
            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::armOnTheSide;
        }
    }
}
