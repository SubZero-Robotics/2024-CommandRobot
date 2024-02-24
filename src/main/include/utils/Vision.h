#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
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
    auto camResult = camera.GetLatestResult();

    std::optional<photon::EstimatedRobotPose> visionEst;

    if (camResult.HasTargets() &&
        (camResult.targets.size() > 1 ||
         camResult.targets[0].GetPoseAmbiguity() <= maxAbmiguity)) {
      visionEst = est.Update(camResult);
    }

    if (visionEst.has_value()) {
      auto estimatedPose = visionEst.value().estimatedPose;
      // Replace 100_m with actual field dimensions
      if (estimatedPose.X() > 0_m && estimatedPose.X() <= 100_m &&
          estimatedPose.Y() > 0_m && estimatedPose.Y() <= 100_m) {
        return visionEst;
      }
    }

    return std::nullopt;
  }

  // See:
  // https://github.com/Hemlock5712/2023-Robot/blob/Joe-Test/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
  void UpdateEstimatedGlobalPose(frc::SwerveDrivePoseEstimator<4U>& estimator) {
    auto cam1Pose =
        GetPoseFromCamera(frc::Pose3d(estimator.GetEstimatedPosition()),
                          photonEstimator, *camera);

    if (cam1Pose.has_value()) {
      auto est = cam1Pose.value();
      AddVisionMeasurement(est, estimator);
    }

    auto cam2Pose =
        GetPoseFromCamera(frc::Pose3d(estimator.GetEstimatedPosition()),
                          photonEstimator2, *camera2);

    if (cam2Pose.has_value()) {
      auto est = cam2Pose.value();
      AddVisionMeasurement(est, estimator);
    }
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(
      photon::EstimatedRobotPose& pose) {
    Eigen::Matrix<double, 3, 1> estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : pose.targetsUsed) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            pose.estimatedPose.ToPose2d().Translation());
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
  void AddVisionMeasurement(photon::EstimatedRobotPose& estimate,
                            frc::SwerveDrivePoseEstimator<4U>& estimator) {
    auto stdDevs = GetEstimationStdDevs(estimate);
    wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
    estimator.AddVisionMeasurement(estimate.estimatedPose.ToPose2d(),
                                   estimate.timestamp, newStdDevs);
  }

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