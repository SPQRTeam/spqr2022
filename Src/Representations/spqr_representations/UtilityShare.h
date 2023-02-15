#pragma once


#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"


STREAMABLE(UtilityShare, COMMA public PureBHumanArbitraryMessageParticle<idUtilityShare>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  float x,
  
  
  (int) striker,
  (int) defender,
  (int) jolly, 
  (int) supporter,
  
});