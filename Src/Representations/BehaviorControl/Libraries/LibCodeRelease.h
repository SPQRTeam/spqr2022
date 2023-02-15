/**
 * @file LibCodeRelease.h
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Role.h"

using namespace Eigen;

STREAMABLE(LibCodeRelease,
{
  FUNCTION(bool(float value, float min, float max)) between;
  FUNCTION(float(float x, float y)) norm;
  FUNCTION(bool(float currentValue, float target, float bound)) isValueBalanced;
  FUNCTION(float(float x, float y)) angleToTarget;

  FUNCTION(Pose2f(float x, float y)) glob2Rel;
  FUNCTION(Pose2f(int cell)) disambiguateCell;

  FUNCTION(float(float x))radiansToDegree;
  FUNCTION(int(float x, float y)) discretizePose;

  FUNCTION(bool(int myPose, Vector2f ballPose)) otherStateHasBall;
  FUNCTION(bool(Pose2f myPose, Vector2f ballPose)) stateHasBall;
  FUNCTION(std::vector<Obstacle>(Pose2f myPose, std::vector<Obstacle> oldOpponents)) globalizeObstacles;
  FUNCTION(std::vector<float>(Pose2f myPose, std::vector<Obstacle> opponents)) computeFreeAreas;
  FUNCTION(bool(Pose2f myPose, Vector2f ballPose, std::vector<Obstacle> opponents)) freeBallSight;

  FUNCTION(std::vector<float>(std::vector<float> freeAreas)) computeTarget;
  FUNCTION(Pose2f(float target, Pose2f robotPose)) computeBetterTarget;

  FUNCTION(Pose2f(Pose2f target, Pose2f globBall, bool behind)) approachPoint;

  FUNCTION(bool(Pose2f targetPose, Pose2f shootingPose,std::vector<Obstacle> opponents)) canPass;
  FUNCTION(Pose2f(bool kickoff, Role::RoleType rRole)) getReadyPose;
  FUNCTION(Pose2f()) activeOpponent;

  FUNCTION(Pose2f(float x, float y)) rel2Glob;
  FUNCTION(float(Pose2f p1, Pose2f p2)) distance;
  FUNCTION(float()) defenderDynamicY;
  FUNCTION(float()) defenderDynamicDistance;
  FUNCTION(bool()) defenderNearestBall;
  FUNCTION(Pose2f()) nearestTemmate;

  // passing the ball
  FUNCTION(bool(std::vector<Obstacle>, Vector2f& target, const Vector2f& translation)) findPassingLineAux;
  FUNCTION(Vector2f(Vector2f target, std::vector<Vector2f> mates)) findPassingLine;
  FUNCTION(Pose2f()) poseToReceive;
  FUNCTION(Pose2f()) poseToPass;

  FUNCTION(Vector2f()) nearestOpponentToBall;
  FUNCTION(bool(Vector2f point)) obstacleExistsAroundPoint;
  FUNCTION(float(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2)) distanceToLine;
  FUNCTION(Vector2f()) getJollyPosition;
  FUNCTION(bool()) opponentOnOurField;
  FUNCTION(std::tuple<int,int,Pose2f>()) strikerPassShare;
  FUNCTION(Vector2f()) getSupporterMarkPosition;
  FUNCTION(Vector2f()) getSupportStrikerPosition;
  FUNCTION(Vector2f()) getSupporterPosition;
  FUNCTION(Vector2f()) getOwnFieldDefSideKickPos;
  FUNCTION(Vector2f()) getOppFieldDefSideKickPos;
  FUNCTION(std::tuple<Vector2f,Vector2f>()) getOwnFieldOffSideKickPos;
  FUNCTION(std::tuple<Vector2f,Vector2f,bool>()) getOppFieldOffSideKickPos;

  FUNCTION(Pose2f(std::vector<float> freeAreas)) biggestFreeArea;

  FUNCTION(Vector3f(Pose2f pa, Pose2f pb, Pose2f pc)) getTriang,

  (float) angleToGoal,
  (int) timeSinceBallWasSeen,

  (bool) isGoalieInStartingPosition,
  (bool) isGoalieInAngle,
  (bool) isBallInKickAwayRange,
  (bool) isGoalieInKickAwayRange,
  (bool) isBallInArea,
  (bool) isGoalieInArea,
  (float) getGoalieCoverAngleDisplacement,
  (Pose2f) getGoalieCoverPosition,
  (float) goalie_displacement,
  (float) angleToMyGoal,
  (float) penaltyAngle,
  (float) kickAngle,
  (float) correctionKickAngle,
  (bool) ballOutOnLeft,
  (bool) diveBool,

  (float) angleForDefender,
  (float) angleForSupporter,
  (float) angleForJolly,
  (int) isTargetToPass,

  (Vector2f) goaliePosition,
  (Vector2f) defenderPosition,
  (Vector2f) supporterPosition,
  (Vector2f) jollyPosition,
  (Vector2f) searcer_1Position,
  (Vector2f) searcer_2Position,
  (Vector2f) searcer_3Position,
  (Vector2f) searcer_4Position,
  (Pose2f) absBallPosition,
  (std::vector<Vector2f>) placesToSearchBall,
});
