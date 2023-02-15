/**
 * @file LibCodeReleaseProvider.h
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Tools/Math/Eigen.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/FreeCorridors.h"
#include "Representations/spqr_representations/OurDefinitions.h"

MODULE(LibCodeReleaseProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),
  REQUIRES(TeamPlayersModel),
  REQUIRES(FreeCorridors),
  REQUIRES(RobotInfo),
  REQUIRES(ObstacleModel),
  USES(Role),
  USES(RoleAndContext),
  PROVIDES(LibCodeRelease),
});

class LibCodeReleaseProvider : public LibCodeReleaseProviderBase
{
  public:LibCodeReleaseProvider();

  void update(LibCodeRelease& libCodeRelease);
  bool between(float value, float min, float max);
  bool isValueBalanced(float currentValue, float target, float bound);
  float angleToTarget(float x, float y);
  float norm(float x, float y);
  Pose2f glob2Rel(float x, float y);  // TODO MARCO FIX
  Pose2f rel2Glob(float x, float y);
  float distance(Pose2f p1, Pose2f p2);

  bool isGoalieInStartingPosition();
  bool isBallInKickAwayRange();
  bool isGoalieInKickAwayRange();
  bool isBallInArea();
  bool isGoalieInArea();
  bool isGoalieInAngle();
  float goalie_displacement;
  float angleToGoal;
  float angleToMyGoal;
  float penaltyAngle;
  float kickAngle;
  float correctionKickAngle;
  bool ballOutOnLeft;
  int discretizePose(float x, float y);
  float radiansToDegree(float x);
  int isTargetToPass;

  Vector2f nearestOpponentToBall();
  bool obstacleExistsAroundPoint(Vector2f point);
  float distanceToLine(Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2);
  Vector2f getJollyPosition();
  bool opponentOnOurField();
  std::tuple<int,int,Pose2f> strikerPassShare();
  Vector2f getSupporterMarkPosition();
  Vector2f getSupportStrikerPosition();
  Vector2f getSupporterPosition();
  Vector2f getOwnFieldDefSideKickPos();
  Vector2f getOppFieldDefSideKickPos();
  std::tuple<Vector2f,Vector2f> getOwnFieldOffSideKickPos();
  std::tuple<Vector2f,Vector2f,bool> getOppFieldOffSideKickPos();
  Pose2f biggestFreeArea(std::vector<float> freeAreas);
  Pose2f computeBetterTarget (float target, Pose2f robotPose);

  Vector2f updateDefender();
  Vector2f updateSupporter();
  Vector2f updateGoalie();

  Vector3f getTriang3Points(Pose2f pa, Pose2f pb, Pose2f pc);
  std::vector<Vector2f> pointsToSearchBall;
  void getPointsToSearch();

  private:
    float m;
    float q;
    //number of obstacles stored in the past
    uint pastObstacles = 0;
    bool getNewObstacles();
};
