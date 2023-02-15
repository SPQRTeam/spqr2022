//Stop and scan. Scan and stop. // head and body rotation
option(SearchBallHead, (const int) direction)  //direction: -1 for conterclockwise and 1 for clockwise
{
    initial_state(zero)
    {
        transition
        {
            if (state_time > 800) goto one;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(0);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(one)
    {
        transition
        {
            if (state_time > 800) goto twoFinal;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(direction*22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(twoFinal)
    {
        transition
        {
            if (state_time > 800) goto two;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(direction*60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(two)
    {
        transition
        {
            if (state_time > 1000) goto three;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(direction*120);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(three)
    {
        transition
        {
            if (state_time > 800) goto four;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(direction*22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(four)
    {
        transition
        {
            if (state_time > 800) goto five;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(0);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(five)
    {
        transition
        {
            if (state_time > 800) goto sixFinal;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-direction*22);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(sixFinal)
    {
        transition
        {
            if (state_time > 800) goto six;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-direction*60);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }

    state(six)
    {
        transition
        {
            if (state_time > 1000) goto seven;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-direction*120);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            Stand();
        }
    }
    state(seven)
    {
        transition
        {
            if (state_time > 1000) goto turn;
        }

        action
        {
            theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
            theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
            theHeadMotionRequest.pan = Angle::fromDegrees(-direction*40);
            theHeadMotionRequest.tilt = Angle::fromDegrees(40);
            theHeadMotionRequest.speed = pi; //fromDegrees(200);
            // WalkAtRelativeSpeed(Pose2f(direction*1.f, 0.f, 0.f));
        }
    }
    state(turn)
    {
        transition
        {
            if (state_time > 2100) goto zero;
        }

        action
        {
            WalkAtRelativeSpeed(Pose2f(22*1.f, 0.f, 0.f));
        }
    }
};
