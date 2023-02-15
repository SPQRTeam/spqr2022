#include "SPQRPlannerDWK.h"

#include <unistd.h>
#include <iostream>
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"


SPQRPlannerDWK::SPQRPlannerDWK(){


    SPQR::ConfigurationParameters();
}


float calculateNorm(float x1, float y1, float x2, float y2){
    float res = (x1 - x2) * (x1 - x2) + (y1 -y2) * (y1 - y2);
    res = (float)sqrt(res);
    return res;
}


//TODO Implementare divisione in griglia tramite mixture di Gaussiane e copertura con quadrati di grandezza variabile in base a questo.
//TODO per Robocup



void SPQRPlannerDWK::update(SPQRInfoDWK& rep) {
/*
 *
 *     RECTANGLE2("representation:ObstacleModel:rectangle", frontRight, obstacleRadius * 2, obstacleRadius * 2, -robotRotation, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, color);
 *
 */

    // float ballDistance;
    // GOAL POSITION ( TODO check backside point )
    float shoulder_x = 0;
    float leftShoulder_y = +150;
    float rightShoulder_y = -150;

    float xOpponent = 4500; //- theRobotPose.translation.x() - cos(theRobotPose.rotation);
    float yLeft = 750 ;//- theRobotPose.translation.y() - sin(theRobotPose.rotation) ;
    float yRight = -750 ;// - theRobotPose.translation.y() - sin(theRobotPose.rotation);
    Pose2f leftRelOpponent = theLibCodeRelease.glob2Rel(xOpponent,yLeft);
    Pose2f rightRelOpponent = theLibCodeRelease.glob2Rel(xOpponent,yRight);

    DECLARE_DEBUG_DRAWING("representation:ObstacleModel:rectangle", "drawingOnField");
    LINE("representation:ObstacleModel:rectangle", shoulder_x, leftShoulder_y , leftRelOpponent.translation.x(), leftRelOpponent.translation.y(), 20, Drawings::solidPen, ColorRGBA::yellow);
    LINE("representation:ObstacleModel:rectangle", shoulder_x, rightShoulder_y , rightRelOpponent.translation.x(), rightRelOpponent.translation.y(), 20, Drawings::solidPen, ColorRGBA::yellow);

    rep.obstacles = theLibCodeRelease.globalizeObstacles(theRobotPose, theObstacleModel.obstacles);


    ////////////DEBUG////////////
    // Pose2f e = theLibCodeRelease.angleAreaToPass(theObstacleModel.obstacles);   // to check angleAreaToPass
    Pose2f e = theLibCodeRelease.poseToPass();          // to check poseToPass
    Pose2f pointEnd = theLibCodeRelease.glob2Rel( e.translation.x(), e.translation.y() );
    LINE("representation:ObstacleModel:rectangle", 0, 0
            , pointEnd.translation.x(), pointEnd.translation.y(), 80, Drawings::solidPen, ColorRGBA::violet);


    for(unsigned i=0; i<theObstacleModel.obstacles.size() ;i++){
        if(theObstacleModel.obstacles.at(i).isTeammate()) {
            CIRCLE("representation:ObstacleModel:rectangle" , theObstacleModel.obstacles.at(i).center.x(), theObstacleModel.obstacles.at(i).center.y(),
         150 , 20 ,  Drawings::dashedPen , ColorRGBA::black , Drawings::solidBrush , ColorRGBA::green  );

        }
        else{
            CIRCLE("representation:ObstacleModel:rectangle" , theObstacleModel.obstacles.at(i).center.x(), theObstacleModel.obstacles.at(i).center.y(),
         150 , 20 ,  Drawings::dashedPen , ColorRGBA::black , Drawings::solidBrush , ColorRGBA::black  );

        }


    }


     for(int i=0; i<19;i++)
        {
           Pose2f pointStart = theLibCodeRelease.glob2Rel(-4500+(500*i), -3000);
               Pose2f pointEnd= theLibCodeRelease.glob2Rel(-4500+(500*i), 3000);
           LINE("representation:ObstacleModel:rectangle", pointStart.translation.x(), pointStart.translation.y()
            , pointEnd.translation.x(), pointEnd.translation.y(), 20, Drawings::solidPen, ColorRGBA::black);

        }

    for(int i=0; i<12;i++)
        {
           Pose2f pointStart = theLibCodeRelease.glob2Rel(-4500, -3000+(500*i));
               Pose2f pointEnd= theLibCodeRelease.glob2Rel(4500, -3000+(500*i));
           LINE("representation:ObstacleModel:rectangle", pointStart.translation.x(), pointStart.translation.y()
            , pointEnd.translation.x(), pointEnd.translation.y(), 20, Drawings::solidPen, ColorRGBA::black);

        }

    //Update robot pose informations
    rep.IHaveTheBall = theLibCodeRelease.stateHasBall(theRobotPose, theTeamBallModel.position);

    rep.robotPose = theLibCodeRelease.discretizePose(theRobotPose.translation.x(), theRobotPose.translation.y());
    //std::cout<<rep.robotPose<<std::endl;
    rep.robotRotation = theRobotPose.rotation;

    //Update global ball informations
    rep.agreedBallPosition = theTeamBallModel.position;
    rep.ballTimeWhenLastSeen = theTeamBallModel.timeWhenLastSeen;

    //Update other Informations



    // int i = 0;

    //std::cout<<theTeamBallModel.position.x()<<" "<<theTeamBallModel.position.y()<<std::endl;

    // float freePercentage = 0.;
    std::vector<float> freeAreas;




    std::vector<Obstacle> opponents = rep.obstacles;


    if(theObstacleModel.obstacles.size() > 0){
        freeAreas =  theLibCodeRelease.computeFreeAreas(theRobotPose, opponents);


    }else{
        freeAreas.push_back(730.);
        freeAreas.push_back(-730.);
    }

    std::vector<float> targetData = theLibCodeRelease.computeTarget(freeAreas);

    Pose2f betterTarget = theLibCodeRelease.computeBetterTarget(targetData.at(0), theRobotPose);

    rep.betterTarget = betterTarget;
    rep.freeGoalPercentage = targetData.at(1);
    //std::cout<<" area libera"<<rep.freeGoalPercentage<<std::endl;

    //std::cout<<"betterTarget "<<betterTarget.translation.x()<<" "<<betterTarget.translation.y()<<std::endl;
    // Transform target point coordinate into relative for plotting
    Pose2f target_point_rel = theLibCodeRelease.glob2Rel(betterTarget.translation.x(),betterTarget.translation.y());
    LINE("representation:ObstacleModel:rectangle", 0, 0 , target_point_rel.translation.x(), target_point_rel.translation.y(), 20, Drawings::solidPen, ColorRGBA::green);
    // plotting all areas
    for (std::vector<float>::const_iterator i = freeAreas.begin(); i != freeAreas.end(); ++i){
      Pose2f areas_point_rel = theLibCodeRelease.glob2Rel(4500,*i);
      LINE("representation:ObstacleModel:rectangle", 0, 0 , areas_point_rel.translation.x(), areas_point_rel.translation.y(), 20, Drawings::solidPen, ColorRGBA::blue);
    }


    switch(theRole.role){

        case Role::undefined: rep.role = SPQRInfoDWK::undefined; break;
        case Role::goalie: rep.role = SPQRInfoDWK::goalie; break;
        case Role::striker: rep.role = SPQRInfoDWK::striker; break;
        case Role::defender: rep.role = SPQRInfoDWK::defender; break;
        case Role::supporter: rep.role = SPQRInfoDWK::supporter; break;
        case Role::jolly: rep.role = SPQRInfoDWK::jolly; break;
        case Role::penaltyStriker: rep.role = SPQRInfoDWK::penaltyStriker; break;
        case Role::penaltyKeeper: rep.role = SPQRInfoDWK::penaltyKeeper; break;
        case Role::searcher_1: rep.role = SPQRInfoDWK::searcher_1; break;
        case Role::searcher_2: rep.role = SPQRInfoDWK::searcher_2; break;
        case Role::searcher_3: rep.role = SPQRInfoDWK::searcher_3; break;
        case Role::searcher_4: rep.role = SPQRInfoDWK::searcher_4; break;
        case Role::none: rep.role = SPQRInfoDWK::none; break;
    }


}

MAKE_MODULE(SPQRPlannerDWK, behaviorControl)
