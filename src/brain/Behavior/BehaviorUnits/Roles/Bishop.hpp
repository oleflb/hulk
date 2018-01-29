#pragma once
#include "Behavior/Units.hpp"

ActionCommand bishop(const DataSet& d)
{
  const Vector2f relBallPosition = d.robotPosition.fieldToRobot(d.teamBallModel.position);
  const float relBallAngle = std::atan2(relBallPosition.y(), relBallPosition.x());
  const Pose relPlayingPose = Pose(d.robotPosition.fieldToRobot(d.bishopPosition.position), relBallAngle);
  return walkToPose(d, relPlayingPose, false, WalkMode::PATH, Velocity(), 5).combineHead(trackBall(d));
}
