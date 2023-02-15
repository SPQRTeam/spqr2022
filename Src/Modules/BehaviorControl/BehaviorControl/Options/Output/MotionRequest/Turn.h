#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

option(Turn, (const Pose2f&) target)
{   
    float angle = theLibCodeRelease.angleToTarget(target.translation.x(), target.translation.y());
    Pose2f relTarget = theLibCodeRelease.glob2Rel(target.translation.x(), target.translation.y());
    DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");

    LINE("representation:ObstacleModel:rectangle", 0, 0 , relTarget.translation.x(), relTarget.translation.y(), 20, Drawings::solidPen, ColorRGBA::black);

    initial_state(start)
    {
        transition
        {   

            if(std::abs(angle) > Angle::fromDegrees(3.f)){
                goto turnToTarget;
            }else{
                goto targetState;
            }
            
        }
        action
        {   
            lookAtBall();
            //std::cout<<"1"<<std::endl;
            Stand();
        }
    }

    state(turnToTarget)
    {
        transition
        {
            if(std::abs(angle) < Angle::fromDegrees(3.f)){
                
                goto targetState;
            }

        }
        action
        {   
            
            if(angle < 0){
                WalkAtRelativeSpeed(Pose2f(-1.f,0.001f,0.001f));
            }else{
                WalkAtRelativeSpeed(Pose2f(1.f,0.001f,0.001f));
            }
        }
    }


    
    target_state(targetState){
        
    }
}
