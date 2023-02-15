//#define USE_DYNAMIC_POINTS

#ifdef PENALTY_STRIKER_GOALIE
#undef USE_DYNAMIC_POINTS
#endif


option(Kicks, (const std::string) kick)
{
    initial_state(Kicks)
    {
        transition{
            #ifdef USE_DYNAMIC_POINTS
                if (theMotionRequest.kickRequest.mirror)
                {
                    if(theBallModel.estimate.position.y()<80)
                    {
                        DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                       -80,
                                                       -180});
                        theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                    }

                    else{
                        DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                       -theBallModel.estimate.position.y(),
                                                       -180});
                        theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                    }
                }

                else
                {
                    if(theBallModel.estimate.position.y()>-80)
                    {
                        DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                       -80,
                                                       -180});
                        theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                    }

                    else
                    {
                        DynPoint dynTemp(2,4,Vector3f {theBallModel.estimate.position.x(),
                                                       theBallModel.estimate.position.y(),
                                                       -180});
                        theMotionRequest.kickRequest.dynPoints.push_back(dynTemp);
                    }
                }
            #endif

            if(kick == "forwardKick")
                goto forwardKick;
            else if(kick == "fastForwardKick")
                goto fastForwardKick;
            else if(kick == "veryFastForwardKick")
                goto veryFastForwardKick;
            else if(kick == "fastForwardKickShort")
                goto forwardKickShort;
            else if(kick == "fastForwardKickVeryShort")
                goto forwardKickVeryShort;
            else if(kick == "sideKick")
                goto sideKick;
            else if(kick == "lobKick")
                goto lobKick;
            else if(kick == "strongKick")
                goto strongKick;
            else if(kick == "sideWalk")
                goto sideWalk;
            else if(kick == "fastKickBackwards")
                goto fastKickBackwards;
            else if(kick == "pass_2")
                goto pass_2;
            else if(kick == "pass_4")
            goto pass_4;
            else
                throw std::invalid_argument( "received wrong kick ID" );
        }
    }

    state(forwardKick)
    {
        action
        {   
            //std::cout<<"kicks1"<<std::endl;
            theMotionRequest.kickRequest.dynPoints.clear();

            lookAtBall();

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            //            theMotionRequest.kickRequest.dynamical=true;	//TODO CHECK VINCENZO

            //std::cout<<"kicks2"<<std::endl;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickForward;
            //std::cout<<"kicks3"<<std::endl;
        }
    }


    state(fastForwardKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynPoints.clear();
            //lookAtBall();

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

//            theMotionRequest.kickRequest.dynamical=true;

            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickForward;
        }
    }

    state(forwardKickShort)
    {
        action
        {
            theMotionRequest.kickRequest.dynPoints.clear();
            //lookAtBall();

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickForwardShort;
        }
    }

    state(forwardKickVeryShort)
    {
        action
        {
            theMotionRequest.kickRequest.dynPoints.clear();
            //lookAtBall();

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

            theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickForwardVeryShort;
        }
    }

    state(veryFastForwardKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynPoints.clear();

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;


            theMotionRequest.kickRequest.dynPoints.clear();
            theMotionRequest.kickRequest.kickMotionType = KickRequest::veryFastKickForward;
        }
    }



    state(sideKick)
    {
        action
        {
            theMotionRequest.kickRequest.dynPoints.clear();
            theHeadControlMode = HeadControl::lookAtBall;

            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.mirror = theLibCodeRelease.glob2Rel((float)theFieldDimensions.xPosOpponentGroundline,0.f).translation.y() >= .0f ?  false : true;
                theMotionRequest.kickRequest.kickMotionType = KickRequest::sideKick;
            theMotionRequest.kickRequest.dynPoints.clear();

        }
    }
    state(lobKick)
        {
            action
            {
                theMotionRequest.kickRequest.dynPoints.clear();
                theHeadControlMode = HeadControl::lookAtBall;

                theMotionRequest.motion = MotionRequest::kick;
                theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

                theMotionRequest.kickRequest.kickMotionType = KickRequest::lobKick;
                theMotionRequest.kickRequest.dynPoints.clear();

            }
        }
    state(strongKick)
        {
            action
            {
                theMotionRequest.kickRequest.dynPoints.clear();
                theHeadControlMode = HeadControl::lookAtBall;

                theMotionRequest.motion = MotionRequest::kick;
                theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;

                theMotionRequest.kickRequest.kickMotionType = KickRequest::strongKick;
                theMotionRequest.kickRequest.dynPoints.clear();

            }
        }

state(sideWalk)
        {
                action
                        {
                                theMotionRequest.kickRequest.dynPoints.clear();
                        theHeadControlMode = HeadControl::lookAtBall;

                        theMotionRequest.motion = MotionRequest::kick;
                        theMotionRequest.kickRequest.mirror = theLibCodeRelease.glob2Rel((float)theFieldDimensions.xPosOpponentGroundline,0.f).translation.y() >= .0f ?  false : true;
                        theMotionRequest.kickRequest.kickMotionType = KickRequest::sideWalk;
                        theMotionRequest.kickRequest.dynPoints.clear();
                        }
        }


state(fastKickBackwards)
        {
                action
                        {
                                theMotionRequest.kickRequest.dynPoints.clear();
                        theHeadControlMode = HeadControl::lookAtBall;

                        theMotionRequest.motion = MotionRequest::kick;
                        theMotionRequest.kickRequest.mirror = theLibCodeRelease.glob2Rel((float)theFieldDimensions.xPosOpponentGroundline,0.f).translation.y() >= .0f ?  false : true;
                        theMotionRequest.kickRequest.kickMotionType = KickRequest::fastKickBackwards;
                        theMotionRequest.kickRequest.dynPoints.clear();
                        }
        }

state(pass_2)
        {
                action
                        {
                                theMotionRequest.kickRequest.dynPoints.clear();
                        theHeadControlMode = HeadControl::lookAtBall;

                        theMotionRequest.motion = MotionRequest::kick;
                        theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;
                        theMotionRequest.kickRequest.kickMotionType = KickRequest::pass_2;
                        theMotionRequest.kickRequest.dynPoints.clear();
                        }
        }

state(pass_4)
        {
                action
                        {
                                theMotionRequest.kickRequest.dynPoints.clear();
                        theHeadControlMode = HeadControl::lookAtBall;

                        theMotionRequest.motion = MotionRequest::kick;
                        theMotionRequest.kickRequest.mirror = theBallModel.estimate.position.y() <= .0f ?  false : true;
                        theMotionRequest.kickRequest.kickMotionType = KickRequest::pass_4;
                        theMotionRequest.kickRequest.dynPoints.clear();
                        }
        }

}
