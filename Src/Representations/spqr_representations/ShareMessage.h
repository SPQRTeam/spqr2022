
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Representations/Communication/BHumanMessage.h"


STREAMABLE(ShareMessage, COMMA public BHumanMessageParticle<idShareMessage>
{
  /** BHumanMessageParticle functions */
    void operator>>(BHumanMessage& m) const override;

    float x,
    (Vector2f) (Vector2f::Zero()) globBall,
    (int) (0) opponentsX,
    (int) (0) opponentsY,
    (int) (0) batteryLevel,
  
});
