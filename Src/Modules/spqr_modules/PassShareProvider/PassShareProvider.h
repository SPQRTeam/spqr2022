#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/spqr_representations/PassShare.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/spqr_representations/SPQRInfoDWK.h"
#include "Representations/spqr_representations/PossiblePlan.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"



#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/BehaviorControl/Role.h"

#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include <mutex>

MODULE(PassShareProvider,
{,
 USES(BehaviorStatus),
 REQUIRES(BallModel),
 REQUIRES(RobotInfo),
 REQUIRES(LibCodeRelease),
 REQUIRES(TeamData),
 REQUIRES(RobotPose),
 REQUIRES(SPQRInfoDWK),
 REQUIRES(PossiblePlan),
 REQUIRES(Role),
 REQUIRES(FieldDimensions),
 PROVIDES(PassShare),
});

class PassShareProvider : public PassShareProviderBase
{
private:


public:
    void update(PassShare& ps);
    PassShareProvider();
};
