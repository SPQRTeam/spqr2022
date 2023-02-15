
#include "ShareMessage.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

#include <iostream>


void ShareMessage::operator>>(BHumanMessage& m) const
{
  
  m.theBSPLStandardMessage.globBall[0] = globBall.x();
  m.theBSPLStandardMessage.globBall[1] = globBall.y();
  m.theBSPLStandardMessage.opponentsX=opponentsX;
  m.theBSPLStandardMessage.opponentsY=opponentsY;
  m.theBSPLStandardMessage.batteryLevel = batteryLevel;

}
