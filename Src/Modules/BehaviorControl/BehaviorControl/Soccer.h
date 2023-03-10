/**
 * @file Soccer.h
 *
 * This file declares the root behavior option (playing soccer).
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorOption.h"

#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/spqr_representations/PossiblePlan.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/FreeCorridors.h"
#include "Representations/spqr_representations/RoleAndContext.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"

#ifdef __INTELLISENSE__
#define INTELLISENSE_PREFIX Soccer::
#endif
#include "Tools/Cabsl.h"

BEHAVIOR_OPTION(Soccer, BehaviorOptionInterface,
{,
  REQUIRES(BallModel),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraStatus),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(GroundContactState),
  REQUIRES(HeadMotionEngineOutput),
  REQUIRES(JointRequest),
  REQUIRES(KeyStates),
  REQUIRES(LibCodeRelease),
  REQUIRES(MotionInfo),
  REQUIRES(RobotHealth),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(PassShare),
  REQUIRES(PossiblePlan),
  REQUIRES(ObstacleModel),
  REQUIRES(PathPlanner),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamPlayersModel),
  REQUIRES(TeamData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(Role),
  REQUIRES(RoleAndContext),
  REQUIRES(FieldDimensions),
  REQUIRES(FreeCorridors),
  REQUIRES(ObstaclesFieldPercept),

  MODIFIES(ArmMotionRequest),
  MODIFIES(BehaviorStatus),
  MODIFIES(HeadControlMode),
  MODIFIES(HeadMotionRequest),
  MODIFIES(MotionRequest),
});

class Soccer : public SoccerBase, public Cabsl<Soccer>
{
public:
  /** Constructor. */
  Soccer();
private:
  /** Executes the behavior. */
  void execute() override;

#include "Options.h"
};
