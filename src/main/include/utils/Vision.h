#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <limits>
#include <memory>

#include "Constants.h"

using namespace VisionConstants;

class Vision {
 public:
  Vision() {
    photonEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);
  }

  photon::PhotonPipelineResult GetLatestResult() {
    return camera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose() {
    auto visionEst = photonEstimator.Update();
    units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs = kSingleTagStdDevs;
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

 private:
  photon::PhotonPoseEstimator photonEstimator{kTagLayout, kPoseStrategy,
                                              photon::PhotonCamera{kCameraName},
                                              kRobotToCam};
  std::shared_ptr<photon::PhotonCamera> camera{photonEstimator.GetCamera()};
  units::second_t lastEstTimestamp{0_s};
};