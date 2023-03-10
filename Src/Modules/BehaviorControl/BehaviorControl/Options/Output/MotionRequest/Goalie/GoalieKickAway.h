
option(GoalieKickAway)
{
  initial_state(start)
  {
    transition
    {
        if(theBallModel.estimate.position.norm() < 300.f)
            goto alignToGoal;
        else if(theBallModel.estimate.position.norm() > 300.f)
            goto turnToBall;
    }
    action
    {
        Stand();
    }
  }

  state(turnToBall)
  {
	  transition
	  {
          if(theBallModel.estimate.position.norm() < 300.f)
			  goto alignToGoal;

          if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(5.f))
			  goto walkToBall;

	  }
	  action
	  {
		  LookForward();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
	  }
  }

  state(walkToBall)
  {
	transition
	{
        if(theBallModel.estimate.position.norm() < 300.f)
			goto alignToGoal;
	}
	action
    {
        WalkToTarget(Pose2f(50.f,50.f,50.f), theBallModel.estimate.position);
	}
  }

  state(alignToGoal)
  {
	  transition
	  {
		  // angle increased (before 10.f)
#ifdef SPQR_DEBUG_GOALIE
		  std::cerr << "angle to goal: " << std::abs(libCodeRelease.angleToGoal) << ", " << Angle::fromDegrees(80.f) << std::endl;
#endif
          if(std::abs(theLibCodeRelease.angleToGoal) < Angle::fromDegrees(SPQR::GOALIE_KICK_AWAY_ANGLE) && std::abs(theBallModel.estimate.position.y()) < 100.f)
			  goto alignBehindBall;
	  }
	  action
      {
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 300.f, theBallModel.estimate.position.y()));
	  }
  }

  state(alignBehindBall)
  {
	  transition
	  {

#ifdef SPQR_DEBUG_GOALIE
            std::cerr << "first : " << (theLibCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
                  theLibCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f)) << std::endl;
#endif
          if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 30.f, 60.f) ||
                  theLibCodeRelease.between(theBallModel.estimate.position.y(), -60.f, -30.f) )
		  {

#ifdef SPQR_DEBUG_GOALIE
              std::cerr << "second : " <<  (theLibCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
                      std::abs(theLibCodeRelease.angleToGoal) < Angle::fromDegrees(SPQR::GOALIE_KICK_AWAY_ANGLE)) << std::endl;
#endif
              if( theLibCodeRelease.between(theBallModel.estimate.position.x(), 100.f, 170.f) &&
                      std::abs(theLibCodeRelease.angleToGoal) < Angle::fromDegrees(SPQR::GOALIE_KICK_AWAY_ANGLE) ) // angle increased (before 3.f)
				  goto kick;
		  }
	  }
	  action
      {
          if( theBallModel.estimate.position.y() >= .0f )
              WalkToTarget(Pose2f(50.f, 50.f, 50.f),  // before (ang vel 80.f)
                           Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 50.f));
		  else
              WalkToTarget(Pose2f(50.f, 50.f, 50.f), // before (ang vel 80.f)
                           Pose2f(theLibCodeRelease.angleToGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() + 50.f));
	  }
  }


  state(kick)
  {
      transition{
            if (theBallModel.estimate.position.norm()>200 || state_time > 1500.f)
                goto targetState;
      }
	  action
	  {
          lookAtBall();
          Kicks("forwardKick");
	  }
  }
  target_state(targetState){

  }
}
