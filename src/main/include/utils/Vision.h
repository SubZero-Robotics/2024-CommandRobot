#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

#include <limits>
#include <memory>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"

class Vision {
    public:
        Vision() {
            photonEstimator.SetMultiTagFallbackStrategy(
                photonlib::PoseStrategy::LOWEST_AMBIGUITY
            );
        }

        photonlib::PhotonPipelineResult GetLatestResult() {
            return camera->GetLatestResult();
        }

        std::optional<photonlib::EstimatedRobotPose> GetEstimatedGlobalPose() {
            auto visionEst = photonEstimator.Update();
            units::second_t latestTimestamp = camera->GetLatestResult().GetTimestamp();
            bool newResult = units::math::abs(latestTimestamp - lastEstTimestamp) > 1e-5_s;
            if (newResult) {
                lastEstTimestamp = latestTimestamp;
            }
            return visionEst;
        }

        Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
            Eigen::Matrix<double, 3, 1> estStdDevs = VisionConstants::kSingleTagStdDevs;
            auto targets = GetLatestResult().GetTargets();
            int numTags = 0;
            units::meter_t avgDist = 0_m;
            for (const auto& tgt : targets) {
                auto tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
                if (tagPose.has_value()) {
                    numTags++;
                    avgDist += tagPose.value().ToPose2d().Translation().Distance(estimatedPose.Translation());
                }
            }
            if (numTags == 0) {
                return estStdDevs;
            }
            avgDist /= numTags;
            if (numTags > 1) {
                estStdDevs = VisionConstants::kMultiTagStdDevs;
            }
            if (numTags == 1 && avgDist > 4_m) {
                estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                    .finished();
            } else {
                estStdDevs = estStdDevs * ( 1 + (avgDist.value() * avgDist.value() / 30));
            }
            return estStdDevs;
        }
    private:
        photonlib::PhotonPoseEstimator photonEstimator{
            VisionConstants::kTagLayout,
            photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            photonlib::PhotonCamera{VisionConstants::kCameraName}, VisionConstants::kRobotToCam
        };
        std::shared_ptr<photonlib::PhotonCamera> camera{photonEstimator.GetCamera()};
        units::second_t lastEstTimestamp{0_s};
};