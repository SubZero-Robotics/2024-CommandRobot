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
    photonEstimator2.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);
  }

  std::optional<photon::EstimatedRobotPose> GetPoseFromCamera(
      frc::Pose3d prevPose, photon::PhotonPoseEstimator& est,
      photon::PhotonCamera& camera, double maxAbmiguity = 0.2) {
    est.SetReferencePose(prevPose);
    auto visionEst = photonEstimator.Update();
    auto camResult = camera.GetLatestResult();

    auto camMulti = camResult.MultiTagResult().result;

    if ((camMulti.isPresent && camMulti.ambiguity <= maxAbmiguity) ||
        (camResult.HasTargets() &&
         camResult.GetBestTarget().poseAmbiguity <= maxAbmiguity)) {
      return visionEst;
    }

    return std::nullopt;
  }

  std::optional<photon::EstimatedRobotPose> GetEstimatedGlobalPose(
      frc::Pose3d prevPose) {
    auto cam1Pose = GetPoseFromCamera(prevPose, photonEstimator, *camera);

    if (cam1Pose.has_value()) {
      return cam1Pose.value();
    }

    auto cam2Pose = GetPoseFromCamera(prevPose, photonEstimator2, *camera2);

    if (cam2Pose.has_value()) {
      return cam2Pose.value();
    }

    return std::nullopt;
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(
      frc::Pose2d estimatedPose,
      std::span<photon::PhotonTrackedTarget> targets) {
    Eigen::Matrix<double, 3, 1> estStdDevs = kSingleTagStdDevs;
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