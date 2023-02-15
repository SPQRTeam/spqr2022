
#include "PatchesProvider.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <iostream>

#define IS_GOALIE (theRobotInfo.number == 1)

#define MIN_NEIGHBORS (theCameraInfo.camera == CameraInfo::upper) ? MIN_NEIGHBORS_UPPER : MIN_NEIGHBORS_LOWER
#define OBS_NEIGHBORS (theCameraInfo.camera == CameraInfo::upper) ? OBS_NEIGHBORS_UPPER : OBS_NEIGHBORS_LOWER

#define MIN_NEIGHBORS_GOALIE (theCameraInfo.camera == CameraInfo::upper) ? MIN_NEIGHBORS_GOALIE_UPPER : MIN_NEIGHBORS_GOALIE_LOWER
#define OBS_NEIGHBORS_GOALIE (theCameraInfo.camera == CameraInfo::upper) ? OBS_NEIGHBORS_GOALIE_UPPER : OBS_NEIGHBORS_GOALIE_LOWER

MAKE_MODULE(PatchesProvider, perception)

void PatchesProvider::update(SPQRPatches& imagePatches)
{
    DECLARE_DEBUG_DRAWING("representation:BallPercept:image", "drawingOnImage");
    CameraInfo::Camera camera = theCameraInfo.camera;
    // Fill patroling patches only the first time
    if(patrolingPatches.size() == 0){
        if(camera == CameraInfo::upper){
            if(DEBUG)
                std::cerr << "Filling 1st time patroling patches" << std::endl; // just run 1st time
            short x, y, width, height, offset;
            offset = static_cast<short>(UPPER_PATROLING_MARGIN/2);
            width = static_cast<short>((theCameraInfo.width/UPPER_PATROLING_COLS)+UPPER_PATROLING_MARGIN);
            height = static_cast<short>((theCameraInfo.height/UPPER_PATROLING_ROWS)+UPPER_PATROLING_MARGIN);
            for (int i = (UPPER_PATROLING_ROWS-1); i >= 0; i--){
                y = static_cast<short>(std::min(std::min(i*(height/2),theCameraInfo.height-height),std::max(i*height-offset, 0)));
                    // std::cerr << "y = " << y << std::endl;
                for (int j = 0; j < int(UPPER_PATROLING_COLS); j++){
                    SPQRPatch patch;
                    if(!IS_GOALIE){
                        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS);
                    }else{
                        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS_GOALIE);
                    }
                    x = static_cast<short>(std::min(std::min(j*width,theCameraInfo.width-width),std::max(j*width-offset, 0)));
                    patch.offset = Vector2s(FROMBHUMANF(x),FROMBHUMANF(y));//Vector2s(0, FROMBHUMANF(280));
                    patch.width = static_cast<short>(FROMBHUMANF(width));//static_cast<short>(FROMBHUMANF(360));
                    patch.height = static_cast<short>(FROMBHUMANF(height));// static_cast<short>(FROMBHUMANF(200));
                    patrolingPatches.push_back(patch);
                }
            }
        }
    }
   
    if(DEBUG)
        std::cerr << "start frame" << std::endl;
    //> First remove all the old patches
    imagePatches.patches.clear();
    unsigned n_frame_not_seen = std::min(theBallPercept.n_frames_not_seen_up, theBallPercept.n_frames_not_seen_btm);
#ifdef PENALTY_STRIKER_GOALIE
    if (theRobotInfo.number == 1 && camera == CameraInfo::lower)
        return;
#endif
    if (DEBUG)
        std::cerr << "frame not seen " << n_frame_not_seen << std::endl;
    //>>> lastFramePrediction PATCH
    if (theBallPercept.camera == camera && n_frame_not_seen <= NUM_FRAME_LAST_PREDICTION) // seen in the same camera in the last step
    {

        if (DEBUG)
            std::cerr << "lastFramePrediction" << std::endl;
        //> take a ROI around the previous ball
        int roi_range = static_cast<int>(FROMBHUMANF(theBallPercept.radiusInImage * ROI_RANGE_MULTIPLEX));
        SPQRPatch patch;
        patch.offset = Vector2s(static_cast<short>(std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.x()) - roi_range), 0)),
                                static_cast<short>(std::max((int)(FROMBHUMANF(theBallPercept.positionInImage.y()) - roi_range), 0)));
        if(DEBUG){
            printf("roi_range: %d\n offset: %d %d\n",roi_range,patch.offset.x(),patch.offset.y());
        }
        patch.width = static_cast<short>(std::min(roi_range << 1, theImage.CV_image.cols - (int)(FROMBHUMANF(theBallPercept.positionInImage.x())) + roi_range));
        patch.height = static_cast<short>(std::min(roi_range << 1, theImage.CV_image.rows - (int)(FROMBHUMANF(theBallPercept.positionInImage.y())) + roi_range));
        patch.type = SPQRPatch::lastFramePrediction;
        //> if the ROI is on the predicted ball and on an obstacle/own body remove it
        Vector2i center = Vector2i(patch.offset.x() + (patch.width >> 1), patch.offset.y() + (patch.height >> 1));
        if (checkBallPosition(TOBHUMANVI(center)))
        {
            //Vector2f pointInImage;
            if(camera == CameraInfo::upper){
                patch.isHighResolution = UPPER_SCALE_FACTOR == 1;
            }else{
                patch.isHighResolution = LOWER_SCALE_FACTOR == 1;
            }

            computeMinMaxBallDiameter(patch);

            if(!IS_GOALIE){
                if (isOnObstacle(patch)){
                    patch.type = SPQRPatch::PatchType::bottomObstacles;
                    patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS);
                }
                else{
                    patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS);
                }
            }else{
                if (isOnObstacle(patch)){
                    patch.type = SPQRPatch::PatchType::bottomObstacles;
                    patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS_GOALIE);
                }
                else{
                    patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS_GOALIE);
                }
            }
            imagePatches.patches.push_back(patch);
            if (DEBUG)
            {
                std::cerr << "------------------- roi " << std::endl;
                std::cerr << "lastFramePrediction 2" << std::endl;
            }

        }
    }

    if ((n_frame_not_seen > NUM_FRAME_LAST_PREDICTION/2 && n_frame_not_seen < KALMAN_PERSISTENCE) || theTeamBallModel.isValid )
    {
        //>>> kalmanPrediction PATCH
        Vector2f pointInImage;
        if (Transformation::robotToImage(Transformation::fieldToRobot(theRobotPose, theTeamBallModel.position),
                                         theCameraMatrix, theCameraInfo, pointInImage))
        {
            if (DEBUG)
                std::cerr << "kalmanPrediction" << std::endl;
            if(DEBUG){
                    // printf("pointInImagePreComputation %d %d\n",pointInImage.x(),pointInImage.y());
                    // printf("theTeamBallModel.position %d %d\n",theTeamBallModel.position.x(),theTeamBallModel.position.y());
                    Vector2f ret = Transformation::fieldToRobot(theRobotPose, theTeamBallModel.position);
                    printf("Transformation::fieldToRobot %f %f\n",ret.x(),ret.y());
                }
                 
            int ballDiameter = static_cast<int>(FROMBHUMANF(computeBallDiameter(Vector2i(pointInImage.x(), pointInImage.y()))));
            pointInImage = FROMBHUMANVF(pointInImage);
            pointInImage.y() -= ballDiameter >> 1;
            if (pointInImage.x() > 0 && pointInImage.x() < ((camera==CameraInfo::upper) ? UPPER_CAMERA_WIDTH : LOWER_CAMERA_WIDTH) &&
                    pointInImage.y() > 0 && pointInImage.y() < ((camera==CameraInfo::upper) ? UPPER_CAMERA_HEIGHT : LOWER_CAMERA_HEIGHT))
            {
                
                Vector2s min_point = Vector2s(
                  static_cast<short>(std::max((int)(pointInImage.x() - ballDiameter * KALMAN_ROI_SIZE), 0)),
                  static_cast<short>(std::max((int)(pointInImage.y() - (ballDiameter * KALMAN_ROI_SIZE)), 0))
                );
                Vector2s max_point = Vector2s(
                  static_cast<short>(std::min((int)(pointInImage.x() + (ballDiameter * KALMAN_ROI_SIZE)), theImage.CV_image.cols)),
                  static_cast<short>(std::min((int)(pointInImage.y() + (ballDiameter * KALMAN_ROI_SIZE)), theImage.CV_image.rows))
                );
        

                SPQRPatch patch;
                if(!IS_GOALIE){
                      patch = SPQRPatch(
                      theImage,
                      min_point,
                      max_point.x() - min_point.x(),
                      max_point.y() - min_point.y(),
                      SPQRPatch::PatchType::kalmanPrediction,
                      (isOnObstacle(Vector2i(pointInImage.x(), pointInImage.y())) ? static_cast<short>(OBS_NEIGHBORS) : static_cast<short>(MIN_NEIGHBORS))
                    );
                    if (isOnObstacle(patch)){
                        patch.type = SPQRPatch::PatchType::bottomObstacles;
                        patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS);
                    }else{
                        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS);
                    }
                }else{
                    patch = SPQRPatch(
                        theImage,
                        min_point,
                        max_point.x() - min_point.x(),
                        max_point.y() - min_point.y(),
                        SPQRPatch::PatchType::kalmanPrediction,
                        (isOnObstacle(Vector2i(pointInImage.x(), pointInImage.y())) ? static_cast<short>(OBS_NEIGHBORS_GOALIE) : static_cast<short>(MIN_NEIGHBORS_GOALIE))
                    );
                    if (isOnObstacle(patch)){
                        patch.type = SPQRPatch::PatchType::bottomObstacles;
                        patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS);
                    }else{
                        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS);
                    }
                }
                if(camera == CameraInfo::upper){
                    patch.isHighResolution = UPPER_SCALE_FACTOR == 1;
                }else{
                    patch.isHighResolution = LOWER_SCALE_FACTOR == 1;
                }
                    computeMinMaxBallDiameter(patch);
                    imagePatches.patches.push_back(patch);
                    if (DEBUG)
                        std::cerr << "kalmanPrediction 2" << std::endl;
            }
            if(!(n_frame_not_seen > NUM_FRAME_LAST_PREDICTION/2 && n_frame_not_seen < KALMAN_PERSISTENCE) and theTeamBallModel.isValid)
                CROSS("representation:BallPercept:image", TOBHUMANF(pointInImage.x()), TOBHUMANF(pointInImage.y()), 20, 4, Drawings::solidPen, ColorRGBA::violet);
            else
                CROSS("representation:BallPercept:image", TOBHUMANF(pointInImage.x()), TOBHUMANF(pointInImage.y()), 20, 4, Drawings::solidPen, ColorRGBA::orange);
        }
    }

    if(n_frame_not_seen > NUM_FRAME_LAST_PREDICTION + KALMAN_PERSISTENCE/2){
        if (DEBUG)
                std::cerr << "patroling" << std::endl;
       SPQRPatch patch;
        if (getPatrolingPatch(patch, n_frame_not_seen, camera)){   
            patch.type = SPQRPatch::framePatroling;
            if (DEBUG)
                printf("patch:[%d %d]",patch.offset.x(),patch.offset.y());
            // If the ROI we find is outside skip and find another ROI
            int height = theCameraInfo.height - 3;
            int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
            horizon = std::min(horizon, height);
            horizon = static_cast<int>(FROMBHUMANF(horizon));
            if (patch.offset.y() < horizon){
                patch.height = static_cast<short>(std::max(((patch.offset.y() + patch.height) - horizon ),120));
                patch.offset.y() = static_cast<short>(horizon);
                computeMinMaxBallDiameter(patch);
            }else{
                computeMinMaxBallDiameter(patch);
            }
            if(isOnObstacle(patch)){
                patch.type = SPQRPatch::PatchType::bottomObstacles;
                if(!IS_GOALIE){
                    patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS);
                }else{
                    patch.minNeighbors = static_cast<short>(OBS_NEIGHBORS_GOALIE);
                }
            }
            imagePatches.patches.push_back(patch);
        }

    }

}

bool PatchesProvider::isOnObstacle(SPQRPatch& patch){
    // check if the patch contains obstacles
    for(const auto player : theObstaclesImagePercept.obstacles){
        // left side of rectagle of the robot seen 
        if(patch.offset.x() <= static_cast<short>(player.left) && (patch.offset.x()+patch.width) >= static_cast<short>(player.left)){
            return true;
        }
        // Right side of rectagle of the robot seen
        if(patch.offset.x() <= static_cast<short>(player.right) && (patch.offset.x()+patch.width) >= static_cast<short>(player.right)){
            return true;
        }
    }
    return false;
}

bool PatchesProvider::getPatrolingPatch(SPQRPatch& patch, int n_frame_not_seen, CameraInfo::Camera camera)
{
    if( patchIndex == patrolingPatches.size()){
        patchIndex = 0;
    }
    if (camera == CameraInfo::upper){
        //int patchIndex = n_frame_not_seen % (UPPER_PATROLING_COLS*UPPER_PATROLING_ROWS);
        patch = patrolingPatches.at(patchIndex);
        patchIndex++;
    }else{
        patch.offset = Vector2s(0, 0);
        patch.width = static_cast<short>(FROMBHUMANF(theCameraInfo.width));
        patch.height =  static_cast<short>(FROMBHUMANF(theCameraInfo.height));
        patch.isHighResolution = LOWER_SCALE_FACTOR == 1;
    }
    if(!IS_GOALIE){
        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS);
    }else{
        patch.minNeighbors = static_cast<short>(MIN_NEIGHBORS_GOALIE);
    }
    return true;
}

bool PatchesProvider::isOnObstacle(Vector2i ball){
    for(const auto player : theObstaclesImagePercept.obstacles){
        if (ball.x() > player.left && ball.x() < player.right && ball.y() > player.bottom && ball.y() < player.top)
            return true;
    }
    return false;
}

bool PatchesProvider::checkBallPosition(Vector2i ball){
#ifdef TARGET_ROBOT
    // check inside body contour
    if (!theBodyContour.isValidPoint(ball)){
        return false;
    }
#endif
    // calculate the image limits
    int height = theCameraInfo.height - 3;
    int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
    horizon = std::min(horizon, height);
    if (ball.y() <= horizon)
        return false;
    return true;
}

void PatchesProvider::computeMinMaxBallDiameter(SPQRPatch& patch){
    std::vector<int> addCoords = {patch.offset.x(), patch.offset.y(), patch.offset.x() + patch.width, patch.offset.y() + patch.height};
    for (int i = 0; i < 2; ++i){
        const Vector2i& spot = Vector2i(addCoords.at((i << 1) + 0), addCoords.at((i << 1) + 1));
        short diameter = static_cast<short>(FROMBHUMANF(computeBallDiameter(TOBHUMANVI(spot))));
        if (i == 0)
            patch.minBallSize = diameter;
        else
            patch.maxBallSize = diameter;
        CIRCLE("representation:BallPercept:image", TOBHUMANF(addCoords.at((i << 1) + 0)), TOBHUMANF(addCoords.at((i << 1) + 1)), TOBHUMANF((diameter >> 1)),
               3, Drawings::dottedPen, ColorRGBA::blue, Drawings::BrushStyle::noBrush, ColorRGBA::green);
    }
}

short PatchesProvider::computeBallDiameter(Vector2i pointInImage){
    Vector2f correctedStart = theImageCoordinateSystem.toCorrected(pointInImage);
    Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x()
                           - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
    Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
    if(unscaledField.z() >= 0.f){
        return 0;// above horizon
    }
    const float scaleFactor = (theCameraMatrix.translation.z() - theBallSpecification.radius) / unscaledField.z();
    cameraToStart *= scaleFactor;
    unscaledField *= scaleFactor;
    cameraToStart.y() += cameraToStart.y() > 0 ? -theBallSpecification.radius : theBallSpecification.radius;
    cameraToStart /= scaleFactor;
    float appRad = std::abs(theCameraInfo.opticalCenter.x() - cameraToStart.y() - correctedStart.x());
    //return static_cast<short>( appRad ) << 1;
    return static_cast<short>(static_cast<short>( appRad ) << 1);
}
