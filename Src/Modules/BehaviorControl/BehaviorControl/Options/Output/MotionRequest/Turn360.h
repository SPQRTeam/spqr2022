option(Turn360)
{
    initial_state(zero)
    {
        transition
        {
            if(state_time > 9400)
                goto end;
        }
        action 
        {
            LookForward();
            WalkAtRelativeSpeed(Pose2f(1.f, 0.0001f,0.0001f));
        }
    }

    target_state(end){
        transition
        {
            ;
        }
        action
        {
            Stand();
        }
    }
}