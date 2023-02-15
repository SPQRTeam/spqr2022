/**
 * @file LibCodeRelease.cpp
 */
#include "LibCodeReleaseProvider.h"
#include <iostream>

MAKE_MODULE(LibCodeReleaseProvider, behaviorControl);

LibCodeReleaseProvider::LibCodeReleaseProvider(): goalie_displacement(300.f),
        angleToGoal(0.f), angleToMyGoal(0.f), kickAngle(0.f), correctionKickAngle(0.f), ballOutOnLeft(false)
{
    SPQR::ConfigurationParameters();
}

float globBallY;

void LibCodeReleaseProvider::update(LibCodeRelease& libCodeRelease)
{
  libCodeRelease.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  libCodeRelease.angleToGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  libCodeRelease.isGoalieInStartingPosition = isGoalieInStartingPosition();
  libCodeRelease.isBallInArea = isBallInArea();
  libCodeRelease.isGoalieInAngle = isGoalieInAngle();
  libCodeRelease.isGoalieInArea = isGoalieInArea();
  libCodeRelease.isGoalieInKickAwayRange = isGoalieInKickAwayRange();
  libCodeRelease.isBallInKickAwayRange = isBallInKickAwayRange();
  libCodeRelease.between = [&](float value, float min, float max) -> bool
  {
    return value >= min && value <= max;
  };

  libCodeRelease.norm = [&](float x, float y) -> float
  {
    return (float)(sqrt((x*x) + (y*y)));
  };

  libCodeRelease.distance = [&] (Pose2f p1, Pose2f p2)-> float
  {
    float diffX = p1.translation.x() - p2.translation.x();
    float diffY = p1.translation.y() - p2.translation.y();

    return (float)(sqrt((diffX*diffX)+(diffY*diffY)));
  };

  libCodeRelease.biggestFreeArea = [&](std::vector<float> freeAreas) -> Pose2f
    {
        float biggest = 0;
        float left;
        float right;
        for (unsigned i=0; i<freeAreas.size()-1; i=i+2) {
            float temp_left = freeAreas.at(i);
            float temp_right = freeAreas.at(i+1);
            float temp_dist = temp_left-temp_right;
            if (temp_dist > biggest) {
                biggest = temp_dist;
                left = temp_left;
                right = temp_right;
            }
        }
        return Pose2f(left,right);
    };

  libCodeRelease.isValueBalanced = [&](float currentValue, float target, float bound) -> bool
  {
      float minErr = currentValue - (target - bound);
      float maxErr = currentValue - (target + bound);

      if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
          return true;
      else
          return false;
  };

  libCodeRelease.angleToTarget = [&](float x, float y) -> float
  {
    Pose2f relativePosition = glob2Rel(x,y);
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
  };

  libCodeRelease.disambiguateCell = [&](int cell) -> Pose2f
  {
    Pose2f myPose;
    int myY = (int)(cell/18);
    int myX = (int)(cell%18);
    myPose.translation.x() = ((500.f * myX) + 250.f) - (float)theFieldDimensions.xPosOpponentGroundline;
    myPose.translation.y() = ((500.f * myY) + 250.f) - (float)theFieldDimensions.yPosLeftSideline;

    return myPose;
  };

  libCodeRelease.discretizePose = [&](float x, float y) -> int
  {
    int myX = (int)((x + theFieldDimensions.xPosOpponentGroundline)/500.);
    int myY = (int)((y + theFieldDimensions.yPosLeftSideline)/500.);
    int discretizedPose;
    if(myX %18 == 0 && myX != 0){
      discretizedPose = myX + 18*myY -1;
    }else{
      discretizedPose = myX + 18*myY;
    }

    return discretizedPose;
  };

  libCodeRelease.glob2Rel = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float theta = 0;
      float tempX = x - theRobotPose.translation.x();
      float tempY = y - theRobotPose.translation.y();

      result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
      result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

      return Pose2f(theta , result.x(),result.y());
  };

  libCodeRelease.rel2Glob = [&](float x, float y) -> Pose2f
  {
      Vector2f result;
      float rho = (float)(sqrt((x * x) + (y * y)));

      result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
      result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

      return Pose2f(result.x(),result.y());
  };


/////////////////////////////////////////////ROBA NUOVA/////////////////////////////////
    libCodeRelease.defenderDynamicY = [&]() -> float
    {
        float x2 = theTeamBallModel.position.x();
        float y2 = theTeamBallModel.position.y();
        float x1 = -4500.f;   // first goalpost for defender
        float y1 = (y2/(std::abs(y2)+1))*750.f;   // first goalpost for defender
        float defenderBallY = (( libCodeRelease.defenderPosition.x()-x1 )*( y2-y1 ))/( x2-x1 ) + y1;

        return defenderBallY-(y2/(std::abs(y2)+1))*100.f;
    };

    libCodeRelease.defenderDynamicDistance = [&]() -> float
    {
        float x1 = theRobotPose.translation.x();
        float y1 = theRobotPose.translation.y();
        float x3 = theTeamBallModel.position.x();
        float y3 = theTeamBallModel.position.y();
        float x2 = (float)theFieldDimensions.xPosOwnGroundline;   // first goalpost for defender
        float y2 = (y3/(std::abs(y3)+1))*750.f;  // first goalpost for defender
        float m = (y1-y2)/(x1-x2) ;
        float q = y1 - (((y1-y2)/(x1-x2))*x1) ;


        float distance = std::abs( y3 - (m*x3 +q) )/(std::sqrt( 1 + (m*m) ));

        return distance;
    };

    libCodeRelease.defenderNearestBall = [&]() -> bool
    {
        Vector2f ballPosition = theBallModel.estimate.position;
        float myDistanceToBall = theBallModel.estimate.position.norm();
        bool iAmMostNearPlayer = true;
        for(unsigned i = 0; i < theTeamData.teammates.size(); i++){
            if((theTeamData.teammates.at(i).theRobotPose.translation -
                    theTeamBallModel.position).norm() < (myDistanceToBall+400.f) && theTeamData.teammates.at(i).number != 1){
                iAmMostNearPlayer = false;
            }
        }
        return iAmMostNearPlayer;
    };

    libCodeRelease.nearestTemmate = [&]() -> Pose2f
    {
        Pose2f nearest = Pose2f(4500.f,0.f);
        for(const auto& mate : theTeamData.teammates){
                if( (mate.theRobotPose.translation- theRobotPose.translation).norm() < 500.f) {
                    nearest = mate.theRobotPose.translation;
                }
            }
        return nearest;
    };

    /**
     * Find a free passing line to the target
     * Returns a valid target or goal center if fails
     *
     * The main function findPassingLine uses and auxiliary function
     * to correctly cycle over all opponents.
     */

    libCodeRelease.findPassingLineAux = [&](std::vector<Obstacle> opponents,
                                            Vector2f& target,
                                            const Vector2f& translation
                                            ) -> bool {
      if(opponents.empty()) {
        target = translation;
        return true;
      }

      // compare against translation
      Eigen::ParametrizedLine<float,2> line = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, translation);

      // in case of failure
      std::vector<Obstacle> originalVector = opponents;

      for(auto it=opponents.begin(); it!=opponents.end(); ++it) {
        if(line.distance(it->center) > 400.f) {
          // call this with reduced obstacles
          opponents.erase(it);
          if(libCodeRelease.findPassingLineAux(opponents, target, translation)) {
            target = translation;
            return true;
          }
        }
        opponents = originalVector;
      }
      return false;
    };

    libCodeRelease.findPassingLine = [&](Vector2f target, std::vector<Vector2f> mates) -> Vector2f {
       bool found = false;
       Vector2f originalTarget = target;
       Vector2f leftTarget = target + Vector2f(0,0);
       Vector2f rightTarget = target - Vector2f(0,0);

       // normalization factor for distance w.r.t. the receiver
       // used to avoid throwing the ball on the back of the receiver
       float nobacknormalization = 1;
       bool isThrowingOnTheBack = false;

       // only use opponents ahead
       // or if behind the opponent penalty mark also use those behind
       std::vector<Obstacle> opponents;
       for(const auto& obs : theTeamPlayersModel.obstacles) {
         if(obs.type == Obstacle::opponent) {
           // if theRobot is inside penalty area then push robots from 2000f to theRobot position
           // else push those ahead only
           if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark &&
              obs.center.x() < theRobotPose.translation.x() &&
              obs.center.x() > 2000.f) {
             opponents.push_back(obs);
           } else if(obs.center.x() > theRobotPose.translation.x() &&
              obs.center.x() < target.x()) {
             opponents.push_back(obs);
           }
         }
       }

       // now cycle over the merged obstacles
       while(!found &&
             leftTarget.y() < theFieldDimensions.yPosLeftSideline &&
             rightTarget.y() > theFieldDimensions.yPosRightSideline) {

         // check if we are throwing the ball on the back of the receiver
         Eigen::ParametrizedLine<float,2> toReceiver1 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, leftTarget);
         Eigen::ParametrizedLine<float,2> toReceiver2 = Eigen::ParametrizedLine<float,2>::Through(theRobotPose.translation, rightTarget);
         for(Vector2f mate : mates) {
           // do not do this for back passes
           if(mate.x() < theRobotPose.translation.x()) {
             continue;
           }

           nobacknormalization = 1+(std::abs(mate.x() - theRobotPose.translation.x())/theFieldDimensions.xPosOpponentGroundline);
          if(std::abs(toReceiver1.distance(mate)) < 200.f*nobacknormalization ||
             std::abs(toReceiver2.distance(mate)) < 200.f*nobacknormalization) {
             isThrowingOnTheBack = true;
             break;
           }
         }

         if(isThrowingOnTheBack) {
           // increase left and right
           isThrowingOnTheBack = false;
         } else {
           // compute the actual passage
           if(target.y() > theRobotPose.translation.y()) {
             if(libCodeRelease.findPassingLineAux(opponents, target, rightTarget) ||
                libCodeRelease.findPassingLineAux(opponents, target, leftTarget)) {
               return target;
             }
           } else {
             if(libCodeRelease.findPassingLineAux(opponents, target, leftTarget) ||
                libCodeRelease.findPassingLineAux(opponents, target, rightTarget)) {
               return target;
             }
           }
         }

         // increase left and right
         leftTarget = leftTarget + Vector2f(0,50);
         rightTarget = rightTarget - Vector2f(0,50);
       }
       // maybe next time
       return Vector2f(theFieldDimensions.xPosOpponentGroundline, 0);
    };

    /*
     * This function orderes the team mates according to their distance w.r.t. the opponents
     * and then, starting from the most free one, tries to find a passing line
     * @return a Vector2f poiting to the passing target or the goal line if no passing is found
     */
    libCodeRelease.poseToPass = [&]() -> Pose2f
    {
      // custom team mate definition
      struct eMate {
        Vector2f position;
        float utility;
        int number;
      };

      // generate the list of valid teammates
      std::vector<eMate> orderedMates;
      // generate auxiliary list of vector2f of available mates
      std::vector<Vector2f> auxMates;
      // for rear pass do not consider forward passage, do this only for forward pass
      float passForward = 600.f;

      for(const auto& mate : theTeamData.teammates) {
        // enable rear passage if inside the opponent area
        // or if not robots have to be ahead of us
        if((theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark && mate.theRobotPose.translation.x() > 2000.f)
           || mate.theRobotPose.translation.x() > theRobotPose.translation.x()) {
          eMate emate;
          emate.position = mate.theRobotPose.translation;
          emate.utility = std::numeric_limits<float>::max();
          emate.number = mate.number;

          // order w.r.t. distance from the closest opponent
          for(const auto& obs : theTeamPlayersModel.obstacles) {
            if(obs.type == Obstacle::opponent &&
               obs.center.x() <= mate.theRobotPose.translation.x()) {
              // compute emate utility w.r.t. obs
              float newUtility = (float)(std::pow(emate.position.x()-obs.center.x(),2) +
                std::pow(emate.position.y()-obs.center.y(),2));
              if(newUtility < emate.utility) {
                emate.utility = newUtility;
              }
            }
          }
          // order w.r.t utility
          bool added = false;
          if(orderedMates.empty()) {
            orderedMates.push_back(emate);
            auxMates.push_back(mate.theRobotPose.translation);
          } else {
            for(auto it = orderedMates.begin(); it != orderedMates.end(); ++it) {
              if(it->utility < emate.utility) {
                orderedMates.insert(it, emate);
                added = true;
                break;
              }
            }
          }
          if(!added) {
            // actual ordered list of mates
            // TODO remove to optimize
            orderedMates.push_back(emate);
            // use for the chek of robot backs (see find passing line)
            auxMates.push_back(mate.theRobotPose.translation);
          }
        }
      }

      // now call the findPassingLine starting from the best mate
      for(const auto& mate : orderedMates) {
        if(theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
          passForward = 0.f;
        }
        Vector2f target = libCodeRelease.findPassingLine(mate.position, auxMates);
        // make forward pass
        if(mate.position.x() < theFieldDimensions.xPosOpponentPenaltyMark) {
          // increase the x
          target.x() = target.x()+passForward;
        }

        // if target is the center of the goal then no valid passing line, continue
        if(target != Vector2f(theFieldDimensions.xPosOpponentGroundline+passForward,0.f)){
          libCodeRelease.isTargetToPass = mate.number;
          return target;
        }
      }
      return Vector2f(theFieldDimensions.xPosOpponentGroundline,0.f);
  };

  libCodeRelease.canPass = [&](Pose2f targetPose, Pose2f shootingPose, std::vector<Obstacle> opponents) -> bool
  {
    float m = 0;
    bool sameX = false;
    bool sameY = false;
    //se hanno la stessa x (più o meno)
    if(std::abs(shootingPose.translation.x() - targetPose.translation.x()) <= 0.5){
      sameX = true;
    }
    else{
      float m = shootingPose.translation.y() - targetPose.translation.y();
      if(std::abs(m) <= 0.5 ){
        sameY = true;
      }
      else{
        m /= shootingPose.translation.x() - targetPose.translation.x();
      }
    }
    float q = shootingPose.translation.y() - m * shootingPose.translation.x();
    float qThreshold = 100.f;
    //TODO inserire il vettore degli opponents
    for(auto const& opponent : opponents){
      if(sameX){
        if(std::abs(opponent.center.x() - shootingPose.translation.x()) < qThreshold){
          std::cout<<"1"<<std::endl;
          return false;
        }
      }
      else if(sameY){
        if(std::abs(opponent.center.y() - shootingPose.translation.y()) < qThreshold){
          std::cout<<"2"<<std::endl;
          return false;
        }
      }
      else{
        for(int i = -50; i < 50; i++){
          //se l'opponent si trova su una retta del fascio
          if((opponent.center.y() - m * opponent.center.x() - q - (float) i * 10.f) <= 0.5f ){
            std::cout<<"3"<<std::endl;
            return false;
          }
        }
      }
    }
    return true;

  };


  libCodeRelease.stateHasBall = [&](Pose2f myPose, Vector2f ballPose) -> bool {

      int myCell = discretizePose(myPose.translation.x(),myPose.translation.y());
      int ballCell= discretizePose(ballPose.x(),ballPose.y());

      if(myCell == ballCell || theBallModel.estimate.position.norm() < 500.f ){
          return true;
      }else{
          return false;
      }
  };

  libCodeRelease.globalizeObstacles = [&](Pose2f myPose, std::vector<Obstacle> oldOpponents) -> std::vector<Obstacle> {
      unsigned i;
      std::vector<Obstacle> opponents;
      for(i = 0; i < oldOpponents.size(); i++){
          Pose2f newLeft = rel2Glob(oldOpponents.at(i).left.x(),oldOpponents.at(i).left.y());
          Pose2f newCenter = rel2Glob(oldOpponents.at(i).center.x(),oldOpponents.at(i).center.y());
          Pose2f newRight =  rel2Glob(oldOpponents.at(i).right.x(),oldOpponents.at(i).right.y());
          oldOpponents.at(i).left.x() = newLeft.translation.x();
          oldOpponents.at(i).left.y() = newLeft.translation.y();

          oldOpponents.at(i).center.x() = newCenter.translation.x();
          oldOpponents.at(i).center.y() = newCenter.translation.y();

          oldOpponents.at(i).right.x() = newRight.translation.x();
          oldOpponents.at(i).right.y() = newRight.translation.y();
          if(oldOpponents.at(i).center.x() >  myPose.translation.x()){

              opponents.push_back(oldOpponents.at(i));
          }
      }
      return opponents;
  };

  libCodeRelease.freeBallSight = [&](Pose2f myPose,Vector2f ballPose, std::vector<Obstacle> opponents) -> bool{
    float ballDist = norm(myPose.translation.x() - ballPose.x(), myPose.translation.y() - ballPose.y());
    unsigned i;
    float oppDist;
    bool sameSide;

    for(i = 0; i < opponents.size(); i++){

      if((myPose.translation.x() - ballPose.x()) >= 0 && (myPose.translation.x() - opponents.at(i).center.x()) >= 0){
        sameSide = true;
      } else if((myPose.translation.x() - ballPose.x()) <= 0 && (myPose.translation.x() - opponents.at(i).center.x()) <= 0){
        sameSide = true;
      } else {
        sameSide = false;
      }
      oppDist = norm(myPose.translation.x() - opponents.at(i).center.x(),
        myPose.translation.y() - opponents.at(i).center.y());

      if(oppDist < ballDist && sameSide == true){

        float divX = myPose.translation.x() - ballPose.x();
        if(divX == 0){
          divX = 0.1f;
        }

        float divY = myPose.translation.y() - ballPose.y();
        if(divY == 0){
          divY = 0.1f;
        }

        float numX =  myPose.translation.x() - opponents.at(i).center.x();
        float numY = myPose.translation.y() - opponents.at(i).center.y();

        float rightEqside = (numY/divY) * divX;

        //std::cout<<"numX "<<numX<<" rightEqside"<<rightEqside<<std::endl;
        if(numX > rightEqside -350 && numX < rightEqside + 350){
          return false;
        }
      }
    }
    return true;

  };

   libCodeRelease.computeFreeAreas = [&](Pose2f myPose, std::vector<Obstacle> opponents) -> std::vector<float>{
    Pose2f goalPoints[2];
    goalPoints[0].translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
    goalPoints[1].translation.x() = (float)theFieldDimensions.xPosOpponentGroundline;
    goalPoints[0].translation.y() = 730.;
    goalPoints[1].translation.y() = -730.;

    std::vector<float> leftPoints;
    std::vector<float> rightPoints;
    std::vector<float> freeAreas;

    float div1;
    if((goalPoints[0].translation.x() - myPose.translation.x() ) == 0){
      div1 = 0.1f;
    } else {
      div1 = (float)((goalPoints[0].translation.x() - myPose.translation.x()));
    }

    // y = ((4500 - x1)/(x2 - x1))*(y2 -y1) + y1

    // float firstM = (goalPoints[0].translation.y() - myPose.translation.y() )/div1;
    // float secondM = (goalPoints[1].translation.y() - myPose.translation.y() )/div2;
    unsigned i, k;
    Obstacle swapper;

    for(i = 0; i < opponents.size(); i++){
        for(k = 0; k < opponents.size(); k++){
            if(opponents.at(i).left.y() > opponents.at(k).left.y() ){
                swapper = opponents.at(k);
                opponents.at(k) = opponents.at(i);
                opponents.at(i) = swapper;
            }
        }
    }
    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).left.x() - myPose.translation.x()) == 0){
          div1 = 0.1f;
        } else {
          div1 = (float)(opponents.at(i).left.x() - myPose.translation.x());
        }

        float y = ((theFieldDimensions.xPosOpponentGroundline - myPose.translation.x())/div1)*(opponents.at(i).left.y() - myPose.translation.y()) + myPose.translation.y();
        leftPoints.push_back(y);
    }

    for(i = 0; i < opponents.size(); i++){
        if((opponents.at(i).right.x() - myPose.translation.x()) == 0){
          div1 = 0.1f;
        } else {
          div1 = (float)(opponents.at(i).right.x() - myPose.translation.x());
        }

        float y = ((theFieldDimensions.xPosOpponentGroundline - myPose.translation.x())/div1)*(opponents.at(i).right.y() - myPose.translation.y()) + myPose.translation.y();
        rightPoints.push_back(y);
    }

    //gestisco caso in cui tutti gli obstacle sono fuori dalla porta
    bool noneInside = true;
    float begin = 730.;
    float end = -730.;

    for(i = 0; i < leftPoints.size(); i++){
        //caso 1
        if((leftPoints.at(i) < 730. && leftPoints.at(i) > -730.) || (rightPoints.at(i) < 730. && rightPoints.at(i) > -730.)){
            noneInside = false;
        }
        if(leftPoints.at(i) > 730. && rightPoints.at(i) < 730.){
            begin = rightPoints.at(i);

        }
        if(leftPoints.at(i) > -730. && rightPoints.at(i) < -730.){
            end = leftPoints.at(i);

        }
    }
    if(noneInside == true){
        freeAreas.push_back(begin);
        freeAreas.push_back(end);
        return freeAreas;
    }
    freeAreas.push_back(begin);
    for(i = 0; i < leftPoints.size(); i++){
        if(leftPoints.at(i) < begin && leftPoints.at(i) > end){
            freeAreas.push_back(leftPoints.at(i));
        }
        if(rightPoints.at(i) > end && rightPoints.at(i) < begin){
            freeAreas.push_back(rightPoints.at(i));
        }
    }
    freeAreas.push_back(end);
    return freeAreas;
};

libCodeRelease.activeOpponent = [&]() -> Pose2f
  {
    int minDist = std::numeric_limits<int>::max();
    int dist;
    Pose2f oppPose = Pose2f(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
    for(auto obst : theTeamPlayersModel.obstacles){
      if(obst.type == Obstacle::opponent){
        dist = (int)(distance(theTeamBallModel.position, Pose2f(obst.center)));
        if(dist < minDist){
          minDist = dist;
          oppPose = Pose2f(obst.center);
        }
      }
    }
    return oppPose;
  };

  libCodeRelease.computeTarget = [&](std::vector<float> freeAreas) -> std::vector<float> {
    float interval = 0;
    float maxInterval = 0;
    unsigned maxIndex = 0;
    unsigned i;
    if(freeAreas.size()%2 == 0){
        for(i = 0; i < freeAreas.size() -1; i+=2){
          interval = std::abs(freeAreas.at(i) -  freeAreas.at(i+1));
          if(maxInterval < interval){
            maxInterval = interval;
            maxIndex = i;
          }
        }
    }else{
      for(i = 1; i < freeAreas.size() -1; i+=2){
        interval = std::abs(freeAreas.at(i) -  freeAreas.at(i+1));
        if(maxInterval < interval){
          maxInterval = interval;
          maxIndex = i;
        }
      }
    }

    std::vector<float> datas;
    datas.push_back((freeAreas.at(maxIndex) + freeAreas.at(maxIndex +1))/2);
    float freePercentage = (float)(std::abs((freeAreas.at(maxIndex) - freeAreas.at(maxIndex +1)))/14.6);
    datas.push_back(freePercentage);
    return datas;
};

libCodeRelease.computeBetterTarget = [&](float target, Pose2f robotPose) -> Pose2f {
    float m = (float)(target/(float)theFieldDimensions.xPosOpponentGroundline);
    //double x = robotPose.translation.x() + 1900;
    float x = (float)theFieldDimensions.xPosOpponentGroundline;
    float y = (float)(m*x);
    //std::cout<<"target "<<target<<" m ="<<m<<" x "<<x<<" y "<<y<<std::endl;
    Pose2f betterTarget;
    betterTarget.translation.x() = x;
    betterTarget.translation.y() = y;
    //std::cout<<" bt x "<<betterTarget.translation.x()<<" bt y "<<betterTarget.translation.y()<<std::endl;
    return betterTarget;
};

libCodeRelease.approachPoint = [&](Pose2f target, Pose2f globBall, bool behind) -> Pose2f {
    Pose2f approachP;
    if(behind){
        approachP.translation.x() = globBall.translation.x() - 200;
    }else{
        approachP.translation.x() = globBall.translation.x() + 200;
    }
    approachP.translation.y() = (((approachP.translation.x() - globBall.translation.x())/target.translation.x()-globBall.translation.x())*
        (globBall.translation.y()-target.translation.y())) + globBall.translation.y();
    return approachP;
};

libCodeRelease.getReadyPose = [&](bool kickoff, Role::RoleType rRole) ->Pose2f {
    switch(rRole){
        case 6:
            rRole = Role::RoleType::defender;
            break;
        case 7:
            rRole = Role::RoleType::supporter;
            break;
        case 8:
            rRole = Role::RoleType::striker;
            break;
        case 9:
            rRole = Role::RoleType::jolly;
            break;
    }
    if( rRole == Role::RoleType::goalie )
        return /* glob2Rel*/Pose2f(SPQR::GOALIE_BASE_POSITION_X + 200.f, SPQR::GOALIE_BASE_POSITION_Y);
    else if( rRole == Role::RoleType::striker )
    {
        if(kickoff)
        {
            return /* glob2Rel*/Pose2f(SPQR::STRIKER_KICKOFF_POSITION_X, SPQR::STRIKER_KICKOFF_POSITION_Y);
        }
        else
        {
            return /* glob2Rel*/Pose2f(SPQR::STRIKER_NO_KICKOFF_POSITION_X, SPQR::STRIKER_NO_KICKOFF_POSITION_Y);
        }
    }
    else if( rRole == Role::RoleType::defender )
            return /* glob2Rel*/Pose2f(SPQR::DEFENDER_DEFAULT_POSITION_X, SPQR::DEFENDER_DEFAULT_POSITION_Y);
    else if( rRole == Role::RoleType::supporter )
            return /* glob2Rel*/Pose2f(SPQR::SUPPORTER_DEFAULT_POSITION_X + 900.f, SPQR::SUPPORTER_DEFAULT_POSITION_Y);
    else if( rRole == Role::RoleType::jolly )
        return /* glob2Rel*/Pose2f(SPQR::JOLLY_DEFAULT_POSITION_X, SPQR::JOLLY_DEFAULT_POSITION_Y);
    else
        return glob2Rel(.0f, SPQR::FIELD_DIMENSION_Y);
};


 libCodeRelease.getTriang = [&] (Pose2f pa, Pose2f pb, Pose2f pc) -> Vector3f
 {
  return getTriang3Points(pa, pb, pc);
 };

  libCodeRelease.radiansToDegree = [&](float x) -> float
  {
    return (float)((x*180)/3.14159265358979323846);
  };

// Returns (-fmax,-fmax) on failure
libCodeRelease.nearestOpponentToBall = [&] () -> Vector2f
{
    Obstacle currentlyNearestOpponent;
    float currentlyNearestDistance = std::numeric_limits<float>::max();
    for(const auto& obstacle : theTeamPlayersModel.obstacles){
        if(obstacle.isOpponent()){
            float thisDistance = distance(obstacle.center,theTeamBallModel.position);
            if(thisDistance < currentlyNearestDistance){
                currentlyNearestDistance = thisDistance;
                currentlyNearestOpponent = obstacle;
            }
        }
    }
    if(currentlyNearestDistance < std::numeric_limits<float>::max())
        return currentlyNearestOpponent.center;
    else
        return Vector2f(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());
};

libCodeRelease.obstacleExistsAroundPoint = [&] (Vector2f point) -> bool
{
    for(const auto& obstacle : theObstacleModel.obstacles){
        if((obstacle.center - point).norm() < 400.f){
            return true;
        }
    }
    return false;
};

libCodeRelease.distanceToLine = [&] (Vector2f objectToCheck, Vector2f linePoint1, Vector2f linePoint2) -> float
{
    return std::abs(((linePoint2.y()-linePoint1.y())*objectToCheck.x()) - ((linePoint2.x() - linePoint1.x()) * objectToCheck.y())
    + (linePoint2.x() * linePoint1.y()) - (linePoint2.y() * linePoint1.x())) / ((linePoint1-linePoint2).norm());
};

libCodeRelease.getJollyPosition = [&] () -> Vector2f
{
  // to return
  Vector2f jollyPosition = Vector2f(0,0);

  // take ball position
  Vector2f ballPosition;
  ballPosition = theTeamBallModel.position;

  // decide the side, should be the opposite
  if(ballPosition.y() < 0.f) {
    if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
      jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 2000.f;
    } else {
        jollyPosition.y() = theFieldDimensions.yPosLeftSideline - 1000.f;
    }
  } else {
        if(ballPosition.x() > theFieldDimensions.xPosOpponentPenaltyMark) {
             jollyPosition.y() = theFieldDimensions.yPosRightSideline + 2000.f;
        } else {
    jollyPosition.y() = theFieldDimensions.yPosRightSideline + 1000.f;
  }
        }

  // if the ball is not ahead of the previous jolly position by 1m
  // take previous y
  for(const auto& mate : theTeamData.teammates) {
    if(mate.role == Role::RoleType::jolly) {
      // check jolly position w.r.t. ball
      if(ballPosition.x()+500.f < 0.f) {
        jollyPosition.y() = -1500.f;
      } else if(ballPosition.x() > mate.theRobotPose.translation.x()+1000.f) {
        jollyPosition.y() = mate.theRobotPose.translation.y();
      }
    }
  }

  jollyPosition.x() = theFieldDimensions.xPosOpponentGroundline*0.6f;
  Vector2f translation = Vector2f(0.f, 0.f);
  for(unsigned i=0; i<4; ++i) {
    jollyPosition = jollyPosition+translation;
    // compute best passing line
    Eigen::ParametrizedLine<float,2> toBall = Eigen::ParametrizedLine<float,2>::Through(jollyPosition, ballPosition);

    float minDist = std::numeric_limits<float>::max();
    for(auto obs : theTeamPlayersModel.obstacles) {
      if(obs.center.x() > ballPosition.x() && obs.center.x() < jollyPosition.x()){
        if(obs.isOpponent()) {
          float thisDist = toBall.distance(obs.center);
          if(thisDist < minDist)
            minDist = thisDist;
        }
      }
    }

    if(minDist > 500.f) {
      return jollyPosition;
    }

    // increment translation
    translation.x() = translation.x()-300.f;
  }

  return jollyPosition;
};

libCodeRelease.opponentOnOurField = [&] () -> bool
{
    for(auto const& obs : theTeamPlayersModel.obstacles){
        if(obs.isOpponent() && obs.center.x() < 0.f){
            return true;
        }
    }
    return false;
};

libCodeRelease.strikerPassShare = [&] () -> std::tuple<int,int,Pose2f>
{
    for(const auto& teammate : theTeamData.teammates){
        if(teammate.role == Role::RoleType::striker) {
            return std::tuple<int,int,Pose2f>(teammate.thePassShare.readyPass,teammate.thePassShare.passingTo,teammate.thePassShare.passTarget);
        }
    }
    return std::tuple<int,int,Pose2f>(0,0,theRobotPose);
};

libCodeRelease.getSupporterMarkPosition = [&] () -> Vector2f
{
    Vector2f strikerPosition;
    Vector2f defenderPosition;

    float STAY_BACK_DISTANCE = 800.f;
    if(theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGroundline + 1500)
      STAY_BACK_DISTANCE = 0;
    else if(theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGroundline / 4.f)
      STAY_BACK_DISTANCE = 350.f;

    for(const auto& teammate : theTeamData.teammates){
        if(teammate.role == Role::RoleType::striker) {
            strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
        else if(teammate.role == Role::RoleType::defender) {
            defenderPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
    }

    Vector2f passPointA = theTeamBallModel.position;

    std::vector<Vector2f> PossibleMarkTargets;
    for(auto const& obs : theTeamPlayersModel.obstacles){
        if(obs.isOpponent() && obs.center.x() < 500.f){
            bool skip = false;
            for(auto const& obs2 : theTeamPlayersModel.obstacles){
                if((obs.center-obs2.center).norm() > 500 && (theRobotPose.translation-obs2.center).norm() > 500.f){
                    // if obs2 is in the line of obs1 and passpointA, this is to be not added to marking
                    if(libCodeRelease.distanceToLine(obs2.center,passPointA,obs.center) < 250.f){
                        float maxDist = (passPointA - obs.center).norm();
                        if((obs2.center-obs.center).norm() < maxDist && (obs2.center-passPointA).norm() < maxDist){
                            skip = true;
                            break;
                        }
                    }
                }
            }
            if((obs.center-passPointA).norm() > 800 && !skip)
                PossibleMarkTargets.emplace_back(obs.center);
        }
    }

    Vector2f leastXAboveStriker(std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
    Vector2f leastXBelowStriker(std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

    for(auto const& obs : PossibleMarkTargets){
        if(obs.y() > strikerPosition.y()){
            if(obs.x() < leastXAboveStriker.x())
                leastXAboveStriker = obs;
        }
        else{
            if(obs.x() < leastXBelowStriker.x())
                leastXBelowStriker = obs;
        }
    }

    // If no mark target
    if(leastXAboveStriker.x() == std::numeric_limits<float>::max() && leastXBelowStriker.x() == std::numeric_limits<float>::max()){
        if(std::abs(strikerPosition.y()) < 500.f){
            if(defenderPosition.y() < strikerPosition.y()){
                float theXPos = (strikerPosition.x()+defenderPosition.x())/2;
                float theYPos = (defenderPosition.y()+strikerPosition.y())/2+1800.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
            else {
                float theXPos = (strikerPosition.x()+defenderPosition.x())/2;
                float theYPos = (defenderPosition.y()+strikerPosition.y())/2-1800.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
        }
        else {
            if(strikerPosition.x() - 500.f > -theFieldDimensions.xPosOpponentGroundline){

                float theXPos = strikerPosition.x() - 500.f;
                float theYPos = strikerPosition.y() - 1800.f;

                if(strikerPosition.y() < 0.f)
                    theYPos = strikerPosition.y() + 1500.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }

                return Vector2f(theXPos,theYPos);
            }
            else {
                float theXPos = strikerPosition.x();
                float theYPos = strikerPosition.y() - 1800.f;

                if(strikerPosition.y() < 0.f)
                    theYPos = strikerPosition.y() + 1500.f;

                if(theXPos > 0.f){
                    float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                    theXPos = 0.f;
                    theYPos = theYPos * ratio;
                }
                return Vector2f(theXPos,theYPos);
            }
        }
    }
    // Mark the one below striker
    else if(leastXAboveStriker.x() == std::numeric_limits<float>::max()){
        float theXPos = (leastXBelowStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
        float theYPos = (leastXBelowStriker.y()*7+passPointA.y()*3)/10;

        if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
            theXPos += STAY_BACK_DISTANCE;

        if(theXPos > 0.f){
            float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
            theXPos = 0.f;
            theYPos = theYPos * ratio;
        }
        return Vector2f(theXPos,theYPos);
    }
    // Mark the one above striker
    else if(leastXBelowStriker.x() == std::numeric_limits<float>::max()){
        float theXPos = (leastXAboveStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
        float theYPos = (leastXAboveStriker.y()*7+passPointA.y()*3)/10;

        if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
            theXPos += STAY_BACK_DISTANCE;

        if(theXPos > 0.f){
            float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
            theXPos = 0.f;
            theYPos = theYPos * ratio;
        }

        return Vector2f(theXPos,theYPos);
    }
    // Mark the nearest one
    else {
        if((leastXAboveStriker - theRobotPose.translation).norm() < (leastXBelowStriker - theRobotPose.translation).norm()){
            float theXPos = (leastXAboveStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
            float theYPos = (leastXAboveStriker.y()*7+passPointA.y()*3)/10;

            if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
                theXPos += STAY_BACK_DISTANCE;

            if(theXPos > 0.f){
                float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                theXPos = 0.f;
                theYPos = theYPos * ratio;
            }

            return Vector2f(theXPos,theYPos);
        }
        else{
            float theXPos = (leastXBelowStriker.x()*7 + passPointA.x()*3)/10-STAY_BACK_DISTANCE;
            float theYPos = (leastXBelowStriker.y()*7+passPointA.y()*3)/10;

            if(theXPos < -theFieldDimensions.xPosOpponentGroundline)
                theXPos += STAY_BACK_DISTANCE;

            if(theXPos > 0.f){
                float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
                theXPos = 0.f;
                theYPos = theYPos * ratio;
            }

            return Vector2f(theXPos,theYPos);
        }
    }
};

libCodeRelease.getSupportStrikerPosition = [&] () -> Vector2f
{
    Vector2f strikerPosition;

    for(const auto& teammate : theTeamData.teammates){
        if(teammate.role == Role::RoleType::striker) {
            strikerPosition = Vector2f(teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y());
        }
    }

    float theXPos = strikerPosition.x() - 600.f;
    float theYPos = strikerPosition.y()/2;
    if(std::abs(theYPos) < 600.f){
        if(theYPos < 0.f)
            theYPos = strikerPosition.y() + 600.f;
        else
            theYPos = strikerPosition.y() - 600.f;
    }

    // if we're defending really close to our goal
    if(strikerPosition.x() < 600.f-theFieldDimensions.xPosOpponentGroundline){
        theXPos = strikerPosition.x() + 200.f;
        if(strikerPosition.y() < 0.f)
            theYPos = strikerPosition.y() + 600.f;
        else
            theYPos = strikerPosition.y() - 600.f;
    }

    // If we're attacking, stay at x=0
    if(theXPos > 0){
        float ratio = theFieldDimensions.xPosOpponentGroundline/(theXPos+theFieldDimensions.xPosOpponentGroundline);
        theXPos = 0;
        theYPos = theYPos * ratio;
    }

    return Vector2f(theXPos,theYPos);

};

libCodeRelease.getSupporterPosition = [&] () -> Vector2f
{
    // if the ball is free
    if(theRoleAndContext.ball_holding_Context == 0){
        float myDistanceToBall = (theRobotPose.translation-theTeamBallModel.position).norm() + 1000.f;
        bool amNearest = true;
        for(const auto& teammate : theTeamData.teammates){
            if(!teammate.isGoalkeeper && teammate.number != theRobotInfo.number) {
                if((teammate.theRobotPose.translation-theTeamBallModel.position).norm() < myDistanceToBall){
                    amNearest = false;
                    break;
                }
            }
        }
        if(amNearest)
            return theTeamBallModel.position;
    }

    Vector2f thePosition;

    if(theRoleAndContext.ball_holding_Context == 2 || theRoleAndContext.ball_holding_Context == 3){
        if(theTeamBallModel.position.x() < 0 || libCodeRelease.opponentOnOurField())
            thePosition = libCodeRelease.getSupporterMarkPosition();
        else {
            thePosition = libCodeRelease.getSupportStrikerPosition();
        }
    }
    else{
        if(theTeamBallModel.position.x() < 0 || !libCodeRelease.opponentOnOurField())
            thePosition = libCodeRelease.getSupportStrikerPosition();
        else
            thePosition = libCodeRelease.getSupporterMarkPosition();
    }

    if(!libCodeRelease.obstacleExistsAroundPoint(thePosition))
        return thePosition;
    else {
        if(thePosition.x()-500.f > -theFieldDimensions.xPosOpponentGroundline){
            if(!libCodeRelease.obstacleExistsAroundPoint(Vector2f(thePosition.x()-500.f,thePosition.y())))
                return Vector2f(thePosition.x()-500.f,thePosition.y());
        }
        else if(thePosition.x()+500.f < 0){
            if(!libCodeRelease.obstacleExistsAroundPoint(Vector2f(thePosition.x()-500.f,thePosition.y())))
                return Vector2f(thePosition.x()+500.f,thePosition.y());
        }
        else
            return thePosition;
    }
    return thePosition;
};

  //ADDED
  //TODO insert the position on the field
  libCodeRelease.absBallPosition = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
  globBallY = libCodeRelease.absBallPosition.translation.y();
  libCodeRelease.defenderPosition = updateDefender();
  libCodeRelease.supporterPosition = updateSupporter();
  libCodeRelease.goaliePosition = updateGoalie();
  libCodeRelease.jollyPosition = Vector2f(theFieldDimensions.yPosLeftGoal, theFieldDimensions.yPosRightGoal);
  libCodeRelease.angleForDefender = angleToTarget(libCodeRelease.defenderPosition.x(), libCodeRelease.defenderPosition.y());
  libCodeRelease.angleForSupporter = angleToTarget(libCodeRelease.supporterPosition.x(), libCodeRelease.supporterPosition.y());
  libCodeRelease.angleForJolly = angleToTarget(theFieldDimensions.yPosLeftGoal, theFieldDimensions.yPosRightGoal);
  libCodeRelease.searcer_1Position = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark-400, 1500);
  libCodeRelease.searcer_2Position = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark-400, -1500);
  libCodeRelease.searcer_3Position = Vector2f(theFieldDimensions.xPosOwnPenaltyMark+400,1500);
  libCodeRelease.searcer_4Position = Vector2f(theFieldDimensions.xPosOwnPenaltyMark+400, -1500);

  //Searchers update TODO CHECK AUSTRALIAN TEAM
  if(getNewObstacles())
    getPointsToSearch();
}

float LibCodeReleaseProvider::angleToTarget(float x, float y)
{
    //gets the relative position of the point to go for the robot
    Pose2f relativePosition = glob2Rel(x,y);
    //std::cerr << "y relativa: "<< relativePosition.translation.y() << ", x relativa: "<<relativePosition.translation.x() << std::endl;
    //return radiansToDegree(atan2f(relativePosition.translation.y(), relativePosition.translation.x()));
    return (atan2f(relativePosition.translation.y(), relativePosition.translation.x()));

    //return glob2Rel(x, y).translation.angle();
}

Pose2f LibCodeReleaseProvider::glob2Rel(float x, float y)
{
    Vector2f result;
    float theta = 0;
    float tempX = x - theRobotPose.translation.x();
    float tempY = y - theRobotPose.translation.y();

    result.x() = (float)(tempX * cos(theRobotPose.rotation) + tempY * sin(theRobotPose.rotation));
    result.y() = (float)(-tempX * sin(theRobotPose.rotation) + tempY * cos(theRobotPose.rotation));

    return Pose2f(theta /*deg*/, result.x(),result.y());
}

//returns the degree value of an angle
float LibCodeReleaseProvider::radiansToDegree(float x)
{
  return (float)((x*180)/3.14159265358979323846);
}

Vector2f LibCodeReleaseProvider::updateDefender()
{
    return Vector2f(theFieldDimensions.xPosOwnDropInLine+100, -700.f);
}

Vector2f LibCodeReleaseProvider::updateSupporter()
{
    return Vector2f(-1750.f, +700.f);
}

Vector2f LibCodeReleaseProvider::updateGoalie()
{
    Pose2f globBall = rel2Glob(theBallModel.estimate.position.x(),theBallModel.estimate.position.y());

                    //-5055
    float deltaX = ((theFieldDimensions.xPosOwnGoal - globBall.translation.x()));// * 0.5f);
    float deltaY = ((globBall.translation.y()));// * 0.5f);

         if(deltaX > theFieldDimensions.xPosOwnPenaltyArea)
             deltaX = theFieldDimensions.xPosOwnPenaltyArea - 200 ;

         if (deltaY < theFieldDimensions.yPosRightGoal + 200 )
             deltaY = theFieldDimensions.yPosRightGoal + 200;

         if (deltaY > theFieldDimensions.yPosLeftGoal - 200 )
             deltaY = theFieldDimensions.yPosLeftGoal - 200;

         else if((deltaY < -200 && deltaY > theFieldDimensions.yPosRightGoal + 200) || (deltaY > 200 && deltaY < theFieldDimensions.yPosRightGoal - 200))
            deltaY = deltaY/2;


        //     std::cout << "x: " << deltaX << std::endl;
        //     std::cout << "y: " << deltaY << std::endl;
    //std::cout<< "LIBCODERELEASE" << deltaX << "  "<< deltaY<<std::endl;
     return Vector2f(deltaX, deltaY);
}

Pose2f LibCodeReleaseProvider::rel2Glob(float x, float y)
{
    Vector2f result;
    float rho = (float)(sqrt((x * x) + (y * y)));

    result.x() = (float)(theRobotPose.translation.x() + (rho * cos(theRobotPose.rotation + atan2(y,x))));
    result.y() = (float)(theRobotPose.translation.y() + (rho * sin(theRobotPose.rotation + atan2(y,x))));

    return Pose2f(result.x(),result.y());
}

bool LibCodeReleaseProvider::isGoalieInStartingPosition()
{

   if( isValueBalanced(theRobotPose.translation.x(), SPQR::GOALIE_BASE_POSITION_X+1000, SPQR::GOALIE_POSE_X_TOLLERANCE) &&
            isValueBalanced(theRobotPose.translation.y(), SPQR::GOALIE_BASE_POSITION_Y+1000, SPQR::GOALIE_POSE_Y_TOLLERANCE) )
        return true;
    else
        return false;
}

bool LibCodeReleaseProvider::isValueBalanced(float currentValue, float target, float bound)
{
    float minErr = currentValue - (target - bound);
    float maxErr = currentValue - (target + bound);

    if( std::abs(minErr) < bound*1.2 && std::abs(maxErr) < bound*1.2 )
        return true;
    else
        return false;
}

  Pose2f LibCodeReleaseProvider::biggestFreeArea(std::vector<float> freeAreas)
    {
        float biggest = 0;
        float left;
        float right;
        for (unsigned i=0; i<freeAreas.size()-1; i=i+2) {
            float temp_left = freeAreas.at(i);
            float temp_right = freeAreas.at(i+1);
            float temp_dist = temp_left-temp_right;
            if (temp_dist > biggest) {
                biggest = temp_dist;
                left = temp_left;
                right = temp_right;
            }
        }
        return Pose2f(left,right);
    }
bool LibCodeReleaseProvider::isBallInKickAwayRange()
{
    if( theBallModel.estimate.position.norm() < SPQR::GOALIE_KICK_AWAY_RANGE )
        return true;
    else
        return false;
}

float LibCodeReleaseProvider::distance(Pose2f p1, Pose2f p2){
    float diffX = p1.translation.x() - p2.translation.x();
    float diffY = p1.translation.y() - p2.translation.y();

    return (float)(sqrt((diffX*diffX)+(diffY*diffY)));
}

bool LibCodeReleaseProvider::isBallInArea()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea) && between(gloBall.translation.y(), theFieldDimensions.yPosRightPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea))
        return true;
    else
        return false;
}

bool LibCodeReleaseProvider::between(float value, float min, float max)
{
    return value >= min && value <= max;
}


bool LibCodeReleaseProvider::isGoalieInAngle()
{
    //~ if(isBallInCoverRange())
        //~ if (between(angleToTarget(theBallModel.estimate.position.x(), theBallModel.estimate.position.y()),
                //~ Angle::fromDegrees(-10.f),
                //~ Angle::fromDegrees(10.f) ) )
            //~ return true;
        //~ else
            //~ return false;
    //~ else
        if (between(theRobotPose.rotation, Angle::fromDegrees(-10.f), Angle::fromDegrees(10.f) ))
            return true;
        else
            return false;
}

// AreaX is between -4500 and -3900, areaY is between -1100 and 1100
bool LibCodeReleaseProvider::isGoalieInArea()
{
    if (between(theRobotPose.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea) && between(theRobotPose.translation.y(), theFieldDimensions.yPosRightPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea))
        return true;
    else
        return false;
}

float LibCodeReleaseProvider::norm(float x, float y)
{
  return (float)(sqrt((x*x) + (y*y)));
}


bool LibCodeReleaseProvider::isGoalieInKickAwayRange()
{
    Pose2f gloBall = rel2Glob(theBallModel.estimate.position.x(), theBallModel.estimate.position.y());
    if (between(gloBall.translation.x(), theFieldDimensions.xPosOwnGroundline, theFieldDimensions.xPosOwnPenaltyArea+100) && between(gloBall.translation.y(), theFieldDimensions.yPosRightPenaltyArea-100, theFieldDimensions.yPosLeftPenaltyArea+100))
        return true;
    else
        return false;
}

int LibCodeReleaseProvider::discretizePose(float x, float y){

    int myX = (int)((x + theFieldDimensions.xPosOpponentGroundline)/500.);
    int myY = (int)((y + theFieldDimensions.yPosLeftSideline)/500.);
    int discretizedPose;
    if(myX %18 == 0 && myX != 0){
      discretizedPose = myX + 18*myY -1;
    }else{
      discretizedPose = myX + 18*myY;
    }

    return discretizedPose;
}

Vector3f LibCodeReleaseProvider::getTriang3Points(Pose2f pose_a, Pose2f pose_b, Pose2f pose_c)
{
  Vector3f results;
  Vector2f pa, pb ,pc;
  pa << pose_a.translation.x(),pose_a.translation.y();
  pb << pose_b.translation.x(),pose_b.translation.y();
  pc << pose_c.translation.x(),pose_c.translation.y();
  float a = (pa-pb).norm();
  float b = (pb-pc).norm();
  float c = (pc-pa).norm();


  results << atan2f(a,c), atan2f(a,b), atan2f(b,c);
  return results;

}

void LibCodeReleaseProvider::getPointsToSearch(){
    //if I have no obstacles or if i saw a new obstacle
    if(pastObstacles == 0 || pastObstacles < theTeamPlayersModel.obstacles.size()){
        pointsToSearchBall.erase(pointsToSearchBall.begin(), pointsToSearchBall.end());
        //add last ball seen, REMOVED IN ORDER TO MAKE THE SEARCHER 4 GO BEHIND while the Searcher3 searchs last ball position
        //pointsToSearchBall.push_back(Vector2f(theTeamBallModel.position.x(),theTeamBallModel.position.y()));
        //for each obstacle, we compute the shadow point
        for(auto const obstacle : theTeamPlayersModel.obstacles){
            //discard far obstacles or teammates
            if((theTeamBallModel.position - obstacle.center).norm() > 2000.f || obstacle.isTeammate())
                continue;
            //almost aligned on the x
            if(std::abs(theRobotPose.translation.x()-obstacle.center.x()) < 500.f ){
                if(theRobotPose.translation.y()>=obstacle.center.y()){
                    if(obstacle.center.x() < SPQR::FIELD_DIMENSION_X && obstacle.center.x() > -SPQR::FIELD_DIMENSION_X &&
                        obstacle.center.y() < SPQR::FIELD_DIMENSION_Y && obstacle.center.y() > -SPQR::FIELD_DIMENSION_Y)
                            pointsToSearchBall.push_back(Vector2f(( obstacle.center.x()), (obstacle.center.y() - 500.f)));
                        }
                else{
                    if(obstacle.center.x() < SPQR::FIELD_DIMENSION_X && obstacle.center.x() > -SPQR::FIELD_DIMENSION_X &&
                        obstacle.center.y() < SPQR::FIELD_DIMENSION_Y && obstacle.center.y() > -SPQR::FIELD_DIMENSION_Y)
                            pointsToSearchBall.push_back(Vector2f(( obstacle.center.x()), (obstacle.center.y() + 500.f)));
                    }
                }
            else
            m = (theRobotPose.translation.y()-obstacle.center.y())/(theRobotPose.translation.x()- obstacle.center.x()) ;
            q = theRobotPose.translation.y() - (m*theRobotPose.translation.x()) ;
            //disambiguate the case if the opponent is ahead or behind the Robot
            if(obstacle.center.x() >= theRobotPose.translation.x()){
                if(obstacle.center.x() < SPQR::FIELD_DIMENSION_X && obstacle.center.x() > -SPQR::FIELD_DIMENSION_X &&
                        obstacle.center.y() < SPQR::FIELD_DIMENSION_Y && obstacle.center.y() > -SPQR::FIELD_DIMENSION_Y)
                            pointsToSearchBall.push_back(Vector2f(( obstacle.center.x() + 500.f), (m * ( obstacle.center.x() + 500.f))+ q));
                }
            else{
                if(obstacle.center.x() < SPQR::FIELD_DIMENSION_X && obstacle.center.x() > -SPQR::FIELD_DIMENSION_X &&
                        obstacle.center.y() < SPQR::FIELD_DIMENSION_Y && obstacle.center.y() > -SPQR::FIELD_DIMENSION_Y)
                            pointsToSearchBall.push_back(Vector2f(( obstacle.center.x() - 500.f), (m * ( obstacle.center.x() - 500.f))+ q));
            }
        }
        //add last Dummy point to skip
        pointsToSearchBall.push_back(Vector2f(0.f, 0.f));
        pastObstacles = static_cast<int>(theTeamPlayersModel.obstacles.size());
    }
    //i I don't see ball for enough time, and i'm in Play state
    // else if(theTeamBallModel.timeWhenLastSeen%3000 == 0 && (int)theGameInfo.setPlay == 0 )
    //     pastObstacles = 0;
}

bool LibCodeReleaseProvider::getNewObstacles(){
    if(pastObstacles != theTeamPlayersModel.obstacles.size()){
            pastObstacles = static_cast<uint>(theTeamPlayersModel.obstacles.size());
            return true;
        }
    return false;
}
