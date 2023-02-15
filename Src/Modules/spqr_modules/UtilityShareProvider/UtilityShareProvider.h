
#pragma once

#include <iostream>
#include <set>
#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/spqr_representations/ConfigurationParameters.h"
#include "Representations/spqr_representations/OurDefinitions.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/spqr_representations/UtilityShare.h"


MODULE(UtilityShareProvider, 
{,
 REQUIRES(Role),
 PROVIDES(UtilityShare),
});

class UtilityShareProvider : public UtilityShareProviderBase
{
private:
    
public:
    void update(UtilityShare& us);
    UtilityShareProvider();
};
