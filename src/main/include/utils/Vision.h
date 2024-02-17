#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <limits>
#include <memory>

#include "Constants.h"

using namespace VisionConstants;

struct CameraResults {
  photon::PhotonPipelineResult camera;
  photon::PhotonPipelineResult camera2;

  std::span<photon::PhotonTrackedTarget> GetTargets() {
    auto targets1 = camera.GetTargets();
    auto targets2 = camera2.GetTargets();
    std::vector<photon::PhotonTrackedTarget> result;
    result.reserve(targets1.size() + targets2.size());
    result.insert(result.end(), targets1.begin(), targets1.end());
    result.insert(result.end(), targets2.begin(), targets2.end());
    return std::span<photon::PhotonTrackedTarget>(result);
  }
};

class Vision {
 public:
  Vision() {
    photonEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);
  }

  CameraResults GetLatestResult() {
    return {camera->GetLatestResult(), camera2->GetLatestResult()};
  }

  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose() {
    auto visionEst = photonEstimator.Update();
    auto visionEst2 = photonEstimator2.Update();
    auto camera1ts = camera->GetLatestResult().GetTimestamp();
    auto camera2ts = camera2->GetLatestResult().GetTimestamp();
    units::second_t latestTimestamp = std::max(camera1ts, camera2ts);
    bool newResult =
        units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }

    return camera1ts > camera2ts ? visionEst : visionEst2;
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
  photon::PhotonPoseEstimator photonEstimator{
      kTagLayout, kPoseStrategy, photon::PhotonCamera{kFrontCamera},
      kRobotToCam};
  photon::PhotonPoseEstimator photonEstimator2{
      kTagLayout, kPoseStrategy, photon::PhotonCamera{kRearCamera},
      kRobotToCam2};
  std::shared_ptr<photon::PhotonCamera> camera{photonEstimator.GetCamera()};
  std::shared_ptr<photon::PhotonCamera> camera2{photonEstimator2.GetCamera()};

  units::second_t lastEstTimestamp{0_s};
};