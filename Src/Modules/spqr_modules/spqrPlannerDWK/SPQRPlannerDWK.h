
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/SPQRInfoDWK.h"

#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"


#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(SPQRPlannerDWK, 
{,
 REQUIRES(TeamBallModel),
 REQUIRES(RobotPose),
 REQUIRES(ObstacleModel),
 REQUIRES(FieldDimensions),
 REQUIRES(Role),
 REQUIRES(FallDownState),
 REQUIRES(TeamData),
 REQUIRES(LibCodeRelease),
 PROVIDES(SPQRInfoDWK),
});

class SPQRPlannerDWK : public SPQRPlannerDWKBase
{
private:
    

public:
    

    
    void update(SPQRInfoDWK& rep);
    SPQRPlannerDWK();
};
