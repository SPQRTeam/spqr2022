option(FreeKickDistance)
{
    initial_state(init)
    {
        transition
        {

        }
        action
        {
            Pose2f globBall = theTeamBallModel.position;
            Pose2f result;
            if(globBall.translation.x() < theRobotPose.translation.x()-100 && std::abs(globBall.translation.y() - theRobotPose.translation.y()) < 150){
                if(globBall.translation.y() < theRobotPose.translation.y())
                    result = Pose2f(globBall.translation.x() - 1000, globBall.translation.y()+8000);
                else
                    result = Pose2f(globBall.translation.x() - 1000, globBall.translation.y()-8000);
            }
            else
                result = Pose2f(globBall.translation.x() - 1000, globBall.translation.y());

            if(std::abs(result.translation.x() - theLibCodeRelease.defenderPosition.x()) > 500)
                WalkToTarget(Pose2f( 1.f, 1.f, 1.f), theLibCodeRelease.glob2Rel(result.translation.x(), result.translation.y()));
            else{
                if(globBall.translation.x() >= theRobotPose.translation.x()-100 && std::abs(globBall.translation.y() - theRobotPose.translation.y()) < 150){
                    if(globBall.translation.y() < theRobotPose.translation.y())
                        result = Pose2f(globBall.translation.x() + 2000, globBall.translation.y()+8000);
                    else
                        result = Pose2f(globBall.translation.x() + 2000, globBall.translation.y()-8000);

                    WalkToTarget(Pose2f( 1.f, 1.f, 1.f), theLibCodeRelease.glob2Rel(result.translation.x(), result.translation.y()));
                }
                else{
                    result = Pose2f(result.translation.x() + 2000, globBall.translation.y());
                    WalkToTarget(Pose2f( 1.f, 1.f, 1.f), theLibCodeRelease.glob2Rel(result.translation.x(), result.translation.y()));
                }
            }
        }
    }
}