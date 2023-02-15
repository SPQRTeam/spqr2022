/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "UtilityShare.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"

#include <iostream>

//#define PLOT_SINGE_TSL(name) \
//  PLOT("representation:FieldFeatureOverview:TimeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));
//

void UtilityShare::operator >> (BHumanMessage& m) const
{ 
  
  
  m.theBHumanArbitraryMessage.queue.out.bin << striker;
  m.theBHumanArbitraryMessage.queue.out.bin << defender;
  m.theBHumanArbitraryMessage.queue.out.bin << jolly;
  m.theBHumanArbitraryMessage.queue.out.bin << supporter;
  
  //std::cout<<"role ="<<role<<std::endl;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
}

bool UtilityShare::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());


  
  m.bin >> striker;
  m.bin >> defender;
  m.bin >> jolly;
  m.bin >> supporter;

  return true;
}
