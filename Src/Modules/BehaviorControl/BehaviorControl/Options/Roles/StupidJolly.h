/*
 * basic RoboCup Jolly Behavior
 * @author Matteo Cecchini
*/

//if not commented DEBUG will show some debug prints

//#define DEBUG

#define DISTANCE_FROM_BALL_XY 1000
#define IDEAL_DISTANCE (DISTANCE_FROM_BALL_XY * std::sqrt(2))
#define BALL_GLOB_X  (theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x() , theBallModel.estimate.position.y() ).translation.x())
#define BALL_GLOB_Y  (theLibCodeRelease.rel2Glob(theBallModel.estimate.position.x() , theBallModel.estimate.position.y() ).translation.y())

option(Jolly)
{
  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto optimalPosition;
    }
    action
    {
      lookLeftAndRight();
      Stand();
    }
  }


  state(optimalPosition)
  {
    transition
    {
      if(BALL_GLOB_X < theFieldDimensions.xPosOpponentDropInLine)
      {
        if( (theRobotPose.translation.x() > BALL_GLOB_X) && ( (theBallModel.estimate.position.norm() > (IDEAL_DISTANCE - 200.f) && (theBallModel.estimate.position.norm() < (IDEAL_DISTANCE + 200.f)))))
        {
          goto turnToBall;
        }
      }
      else
      {
        if( ( (theBallModel.estimate.position.norm() > (IDEAL_DISTANCE - 200.f) && (theBallModel.estimate.position.norm() < (IDEAL_DISTANCE + 200.f)))))
        {
          goto turnToBall;
        }
      }
    }
    action
    { 
      if(BALL_GLOB_X < theFieldDimensions.xPosOpponentDropInLine)
      {
        if(BALL_GLOB_Y < 0)
        {
          SPQRWalkTo(Pose2f(1.f, 1.f, 1.f), Pose2f(BALL_GLOB_X+DISTANCE_FROM_BALL_XY, BALL_GLOB_Y+DISTANCE_FROM_BALL_XY ), theLibCodeRelease.angleToGoal);
        }
        else
        {
          SPQRWalkTo(Pose2f(1.f, 1.f, 1.f), Pose2f(BALL_GLOB_X+DISTANCE_FROM_BALL_XY, BALL_GLOB_Y-DISTANCE_FROM_BALL_XY ), theLibCodeRelease.angleToGoal);
        }
      }
      else
      {
        if(BALL_GLOB_Y < 0)
        {
          SPQRWalkTo(Pose2f(1.f, 1.f, 1.f), Pose2f((float) theFieldDimensions.xPosOpponentPenaltyArea, BALL_GLOB_Y+DISTANCE_FROM_BALL_XY ), theLibCodeRelease.angleToGoal);
        }
        else
        {
          SPQRWalkTo(Pose2f(1.f, 1.f, 1.f), Pose2f((float) theFieldDimensions.xPosOpponentPenaltyArea, BALL_GLOB_Y-DISTANCE_FROM_BALL_XY ), theLibCodeRelease.angleToGoal);
        }
      }
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(theBallModel.estimate.position.norm() <= IDEAL_DISTANCE - 200.f && theBallModel.estimate.position.norm() >= IDEAL_DISTANCE + 200.f)
      {
        goto optimalPosition;
      }
      if(std::abs(theBallModel.estimate.position.angle()) < Angle::fromDegrees(10.f))
      {
        goto stand;
      }
    }
    action
    {
      lookAtBall();
      WalkToTarget(Pose2f(.8f, .8f, .8f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
    }
  }

  state(stand)
  {
    transition
    {
      if((theRobotPose.translation.x() < BALL_GLOB_X) || (theBallModel.estimate.position.norm() <= IDEAL_DISTANCE - 300.f || theBallModel.estimate.position.norm() >= IDEAL_DISTANCE + 300.f))
      {
        goto optimalPosition;
      }
      if(std::abs(theBallModel.estimate.position.angle()) > Angle::fromDegrees(10.f))
      {
        goto turnToBall;
      }
    }

    action
    {
      lookAtBall();
      Stand();
    }

  }

}
