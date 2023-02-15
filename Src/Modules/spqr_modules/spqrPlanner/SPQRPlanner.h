
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/SPQRInfoDWK.h"
#include "Representations/spqr_representations/PossiblePlan.h"
#include "Representations/spqr_representations/PassShare.h"

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Communication/TeamData.h"

#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/BehaviorControl/Role.h"


#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(SPQRPlanner, 
{,
 REQUIRES(SPQRInfoDWK),
 REQUIRES(LibCodeRelease),
 REQUIRES(TeamData),
 REQUIRES(RobotInfo),
 REQUIRES(Role),
 USES(PassShare),
 PROVIDES(PossiblePlan),
});

class Stato{
	public:
		int action; 
    float passUtil;
		Vector2f agreedBallPosition;                         
		int robotPose;
		float robotRotation;
        std::vector<Obstacle> obstacles;  //VINCENZO
		float freeGoalPercentage;
		bool IHaveTheBall;
		Pose2f betterTarget;
		float utility;
    int color; //2 verde, 1 arancio, 0 rosso

  		
};

class SPQRPlanner : public SPQRPlannerBase
{
private:
    

public:
    void update(PossiblePlan& pp);
    Vector2f newBallPose(int cell);
    
    float definePassUtility(Stato s0, float parentWeight, float heuristic);
    float jollyUtility(Stato s0, float parentWeight, float heuristic);
    float strikerUtility(Stato s0, float parentWeight, float heuristic);
    Stato completeState(Vector2f agreedBallPosition,int robotPose,float robotRotation,std::vector<Obstacle> obstacles, int action, float parentWeight, Stato prevState);
    bool equalStates(Stato s1, Stato s2);
    std::vector<Stato> exploitState( Stato sp);
    int manhattanDistance(Stato as, Stato ps);
    SPQRPlanner();
};
