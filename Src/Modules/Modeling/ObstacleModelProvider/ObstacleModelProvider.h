/**
 * @file ObstacleModelProvider.h
 *
 * Declaration of a module that combines arm contacts,
 * foot bumper contacts and players percepts into one obstacle model.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootBumperState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"

#include "ObstacleHypothesis.h"

MODULE(ObstacleModelProvider,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FallDownState),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(Odometer),
  REQUIRES(OwnTeamInfo),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(TorsoMatrix),
  PROVIDES(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (int) deleteAfter,                         /**< Delete obstacles that are not seen for deleteAfter amount of milliseconds */
    (Vector2f) pRobotRotationDeviation,        /**< Deviation of the rotation of the robot's torso */
    (Vector2f) pRobotRotationDeviationInStand, /**< Deviation of the rotation of the robot's torso while standing */
    (float) mergeDistance,                     /**< Distance to merge obstacles */
    (float) goalMergeDistance,                 /**< Distance to merge goalposts */
    (unsigned) minPercepts,                    /**< Minimal amount of percepts to become an obstacle */
    (bool) debug,                              /**< Flag for some debug stuff */
    (float) maxVelocity,                       /**< Unit: mm per ms. Obstacle with higher velocity will be removed */
    (int) colorThreshold,                      /**< Only switch color if this threshold is reached */
    (int) uprightThreshold,                    /**< Only switch upright/fallen if this threshold is reached */
    (float) pNp,                               /**< Process noise for position vector */
    (float) pNv,                               /**< Process noise for velocity vector (moving objects) */
    (Vector2f) odoDeviation,                   /**< Percentage inaccuracy of the odometry */
    (unsigned) mergeOverlapTimeDiff,           /**< Merge overlapping models if they are measured. Avoid merging of oscillating obstacles */
    (unsigned) notSeenThreshold,               /**< How many times an obstacle should be seen. If that threshold is reached an obstacle will be removed */
    (unsigned) maxDistance,                    /**< Maximal distance to an obstacle */
    (float) weightedSum,                       /**< Factor for weighted sum for calculation of width of an obstacle */
    (float) cameraAngleFactor,                 /**< Factor for opening angle of the camera */
    (bool) useArmContactModel,
    (bool) useFootBumperState,
    (bool) useTeammatePositionForClassification,
  }),
});

class ObstacleModelProvider : public ObstacleModelProviderBase
{
  static_assert(Obstacle::Type::fallenTeammate > Obstacle::Type::teammate, "Assumption broken");
  static_assert(Obstacle::Type::fallenOpponent > Obstacle::Type::opponent, "Assumption broken");
  static_assert(Obstacle::Type::fallenSomeRobot > Obstacle::Type::someRobot, "Assumption broken");
  static_assert(Obstacle::Type::unknown < Obstacle::Type::someRobot, "Assumption broken");
  static_assert(Obstacle::Type::goalpost < Obstacle::Type::unknown, "Assumption broken");

  // functions to provide the model
  void update(ObstacleModel& obstacleModel) override;
  bool clearAndFinish(ObstacleModel& obstacleModel);

  void addArmContacts(); /**< add obstacles measured by arm contact */
  void addFootContacts(); /**< add obstacles measured by foot contact */
  void addPlayerPercepts(); /**< add players percepts */
  void dynamic(); /**< apply dynamic step from extended kalman filter on all obstacles */
  void deleteObstacles(); /**< delete wrong goal posts and invalid obstacles */
  void considerTeammates(); /**< adjust 'color' of obstacles if we think we found our teammates */
  void propagateObstacles(ObstacleModel& ObstacleModel) const; /**< send valid obstacles to the representation */
  void tryToMerge(const ObstacleHypothesis& obstacle); /**< find matching obstacle to one measurement */
  void mergeOverlapping(); /**< overlapping obstacles will be merged to one obstacle */
  void shouldBeSeen(); /**< decrease seencount of obstacles that should be seen */
  bool shadowedRobots(ObstacleHypothesis* closer, const size_t i, const float cameraAngleLeft, const float cameraAngleRight); /**< used by shouldBeSeen() */
  void setLeftRight();

  //to gain const one has to use finished()
  const Matrix2f feetCov = (Matrix2f() << 900.f, 0.f, 0.f, 6400.f).finished(); /**< 30mm standard deviation and 80mm standard deviation */
  const Matrix2f armCov = (Matrix2f() << 2500.f, 0.f, 0.f, 2500.f).finished(); /**< 50mm standard deviation */

  //to not spam the annotation
  bool armContact[Arms::numOfArms] = { false, false }, footContact[Legs::numOfLegs] = { false, false };

  std::vector<ObstacleHypothesis> obstacleHypotheses; /**< List of obstacles. */
  std::vector<bool> merged; /**< This is to merge obstacles once for every "percept" per frame */
};
