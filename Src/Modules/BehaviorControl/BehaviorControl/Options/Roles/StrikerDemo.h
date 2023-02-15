/** A test striker option without common decision */
option(StrikerDemo)
{
  common_transition
  {
      if ((theKeyStates.pressed[KeyStates::headFront]))
        goto dead;
      else if ((theKeyStates.pressed[KeyStates::headRear]))
        goto standAndLookAtBall;
  }
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theBallModel.estimate.position.angle()) < 5_deg)
        goto walkToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(walkToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theBallModel.estimate.position.norm() < 500.f)
        //goto alignToGoal;
          goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookAtBall;
      WalkToTarget(Pose2f(0.8f, .8f, .8f), theBallModel.estimate.position);
    }
  }

  state(alignToGoal)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(std::abs(theLibCodeRelease.angleToGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
        goto alignBehindBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(.7f, .7f, .7f), Pose2f(/*theLibCodeRelease.angleToGoal*/0.f, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        goto searchForBall;
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f)
      && theLibCodeRelease.between(theBallModel.estimate.position.x(), 155.f, 195.f)
      /*&& std::abs(theLibCodeRelease.angleToGoal) < 2_deg*/)
        goto kick;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(/*theLibCodeRelease.angleToGoal*/0.f, theBallModel.estimate.position.x() - 180.f, theBallModel.estimate.position.y() - 40.f));
    }
  }

//  state(alignBehindBall)
//   {
//     transition
//     {
//       if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
//         goto searchForBall;
//       if(theLibCodeRelease.between(std::abs(theBallModel.estimate.position.y()), 30.f, 60.f)
//       && theLibCodeRelease.between(theBallModel.estimate.position.x(), 150.f, 190.f)
//       /*&& std::abs(theLibCodeRelease.angleToGoal) < 2_deg*/)
//         goto kick;
//     }
//     action
//     {

//       theHeadControlMode = HeadControl::lookForward;
//       if (theBallModel.estimate.position.y() > 0)
//         WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(/*theLibCodeRelease.angleToGoal*/0.f, theBallModel.estimate.position.x() - 175.f, theBallModel.estimate.position.y() + 40.f));
//       else
//         WalkToTarget(Pose2f(0.6f, 0.6f, 0.6f), Pose2f(/*theLibCodeRelease.angleToGoal*/0.f, theBallModel.estimate.position.x() - 175.f, theBallModel.estimate.position.y() - 40.f));
//     }
//   }

  state(kick)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
        goto start;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      //InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
      Kicks("veryFastForwardKick");
    }
  }

  state(searchForBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        goto turnToBall;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
  }
  state(dead)
  {
    transition
    {

        if ((theKeyStates.pressed[KeyStates::headMiddle]))
            goto start;
    }
    action
    {
        SpecialAction(SpecialActionRequest::playDead);
    }
  }
  state(standAndLookAtBall)
  {
      transition
      {
          if ((theKeyStates.pressed[KeyStates::headMiddle]))
              goto start;
      }
      action
      {
            theHeadControlMode = HeadControl::lookAtBall;
            Stand();
      }
  }
}
