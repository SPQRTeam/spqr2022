#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Modeling/ObstacleModel.h"


#include <vector>
#include "Tools/Streams/Enum.h"

STREAMABLE(SPQRInfoDWK,
{
	public:

		ENUM(RoleType,    //SPQR Roles
		  {,
		    undefined,
		    goalie,
		    striker,
		    defender,
		    supporter,
		    jolly,
		    penaltyStriker,
		    penaltyKeeper,
		    searcher_1,
		    searcher_2,
		    searcher_3,
		    searcher_4,
		    none,
		  });
	
		float x,        	/**< The X position of the filtered robot pose */
        (Vector2f) agreedBallPosition,                         /**< The position of the ball in global field coordinates (in mm) */
  		(unsigned)(0) ballTimeWhenLastSeen,              /**< The point of time when the ball was seen the last time by a teammate */
  		(int) robotPose,
  		(float) robotRotation,
        (std::vector<Obstacle>) obstacles,
  		(float) freeGoalPercentage,
  		(int) role,
  		(bool) IHaveTheBall,
  		(Pose2f) betterTarget,
        SPQRInfoDWK() = default;
});

STREAMABLE(SPQRInfoDWKCompressed,
{
public:
  SPQRInfoDWKCompressed() = default;
  SPQRInfoDWKCompressed(const SPQRInfoDWK& mySPQRInfoDWK);
  operator SPQRInfoDWK() const,
	(float) x,
	
});
