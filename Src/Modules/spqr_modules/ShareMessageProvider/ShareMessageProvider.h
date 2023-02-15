
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"

#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/spqr_representations/ShareMessage.h"

#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/RobotHealth.h"

#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(ShareMessageProvider, 
{,
    REQUIRES(TeamBallModel),
    REQUIRES(ObstacleModel),
    REQUIRES(RobotPose),
    REQUIRES(LibCodeRelease),
    REQUIRES(FieldDimensions),
    REQUIRES(RobotHealth),
    PROVIDES(ShareMessage),
});

class ShareMessageProvider : public ShareMessageProviderBase
{
private:
    

public:
    void update(ShareMessage& sm);
    ShareMessageProvider();
};
