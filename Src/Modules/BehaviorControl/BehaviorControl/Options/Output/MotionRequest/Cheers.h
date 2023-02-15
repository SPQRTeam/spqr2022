option(Cheers, (const std::string) cheer)
{
    initial_state(Kicks)
    {
        transition{
            if(cheer == "cheer")
                goto cheer;
            else
                throw std::invalid_argument( "received wrong kick ID" );
        }
    }
//cheer motion made for CC 
state(cheer)
    {
        // transition{
        //     if(state_time >= 1500.f)
        //         goto target_state;
        // }
        action
        {   
            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::cheer;
        }
    }

// state(target_state)
// {

// }

}

