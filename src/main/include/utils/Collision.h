#pragma once

#include <Constants.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <vector>

using namespace CollisionConstants;

namespace Collision {
static frc::Translation2d PerformCollision(
    const Obstacle& obstacle, std::vector<frc::Translation2d> object) {
  for (size_t i = 0; i < obstacle.verticies.size(); i++) {
    const auto& currentVertex = obstacle.verticies[i];
    const auto& nextVertex =
        obstacle.verticies[(i + 1) % obstacle.verticies.size()];

    frc::Translation2d edge = nextVertex - currentVertex;

    // hehe funny geometry class comming back to get me
    frc::Translation2d normal = edge.RotateBy(M_PI / 2.0);

    std::vector<double> objectProjections;
    for (const auto& vertex : object) {
      objectProjections.push_back((vertex - currentVertex).Dot(normal));
    }

    double minProjection =
        *std::min_element(robotProjectiions.begin(), robotProjections.end());
    double maxProjection =
        *std::max_element(robotProjectiions.begin(), robotProjections.end());

    if (minProjection < 0 && maxProjection > 0) {
      // :flushed:
      double penetrationDepth = std::min(-minProjection, maxProjection);
      frc::Translation2d correctionVector = normal * penetrationDepth;

      return correctionVector;
    }
    // no collision, return empty translation
    return frc::Translation2d();
  }
}

static frc::Pose2d AdjustEstimatedPose(const frc::Pose2d& pose) {
  frc::Translation2d robotPosition = pose.Translation;
  double robotRotation = pose.Rotation().Radians().value();

  auto adjustedPose = pose;

  std::vector<frc::Translation2d> robotVerticies{
      frc::Translation2d(-kRobotWidth / 2, -kRobotLength / 2),
      frc::Translation2d(kRobotWidth / 2, -kRobotLength / 2),
      frc::Translation2d(kRobotWidth / 2, kRobotLength / 2),
      frc::Translation2d(-kRobotWidth / 2, kRobotLength / 2),
  };

  for (auto& vertex : robotVerticies) {
    vertex = vertex.RotateBy(robotRotation);
    vertex += robotPosition;
  }

  for (const auto& obstacle: obstacles) {
    frc::Translation2d mtv = PerformCollision(obstacle, robotVerticies);
    adjustedPose = adjustedPose.TranslateBy(mtv);
  }

  units::meter_t x = std::clamp(adjustedPose.Translation().X(), 0, kFieldWidth);
  units::meter_t y = std::clamp(adjustedPose.Translation().Y(), 0, kFieldLength);

  return frc::Pose2d(frc::Translation2d(x, y), adjustedPose.Rotation());
}
}  // namespace Collision