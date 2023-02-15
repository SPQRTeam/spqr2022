#include "ShareMessageProvider.h"

#include <unistd.h>
#include <iostream>
#include <math.h>

#include "Tools/Modeling/Obstacle.h"

ShareMessageProvider::ShareMessageProvider(){
    SPQR::ConfigurationParameters();
}

void ShareMessageProvider::update(ShareMessage& sm) {


    sm.globBall[0]=(std::isnan((double)theTeamBallModel.position.x()))?0:theTeamBallModel.position.x();
    sm.globBall[1]=(std::isnan((double)theTeamBallModel.position.y()))?0:theTeamBallModel.position.y();


    std::vector<Obstacle> obstacles= theLibCodeRelease.globalizeObstacles(theRobotPose, theObstacleModel.obstacles);
    std::vector<Vector2f> opponents(4);
    
    //compute opponents from obstacles
  	std::sort(const_cast<std::vector<Obstacle>&>(obstacles).begin(), const_cast<std::vector<Obstacle>&>(obstacles).end(), [](const Obstacle& a, const Obstacle& b) {return a.center.squaredNorm() < b.center.squaredNorm(); });
    const int numOfObstacles = std::min(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES, static_cast<int>(obstacles.size()));
    obstacles.resize(numOfObstacles);

    int j=0;
    for(int i = 0; i < numOfObstacles; i++){
		if (j<4 && obstacles[i].isOpponent()){
            opponents[j]= obstacles[i].center;
            j++;
		}
    }

    //encode opponents positions in 2 variables
    int x=0, y=0, single_x;
    for (int i=0; i< j; i++){
      single_x= (int) (round( (opponents[i].x() + theFieldDimensions.xPosOpponentFieldBorder) /100));
      if (single_x>99)  single_x=99;
      x= x*100 + single_x;
      y= y*100 + (int)( round( (opponents[i].y() + theFieldDimensions.yPosLeftFieldBorder) /100) );
    }

    sm.opponentsX=x;
    sm.opponentsY=y;

    //battery level
    sm.batteryLevel=theRobotHealth.batteryLevel;

}

MAKE_MODULE(ShareMessageProvider, modeling)
