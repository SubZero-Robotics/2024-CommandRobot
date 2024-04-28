#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Transform3d.h>
#include <photon/PhotonPoseEstimator.h>
#include <units/length.h>
#include <wpi/array.h>

#include <string>

namespace VisionConstants {
static constexpr std::string_view kFrontCamera{"PhotonVision"};
static constexpr std::string_view kRearCamera{"Photonvision2"};
static const frc::Transform3d kRobotToCam2{
    frc::Translation3d{2.147_in, 0_in, 23.369_in},
    frc::Rotation3d{0_deg, -23.461_deg, 180_deg}};
static const frc::Transform3d kRobotToCam{
    frc::Translation3d{5.714_in, 0_in, 23.533_in},
    frc::Rotation3d{0_deg, -23.461_deg, 0_deg}};
constexpr photon::PoseStrategy kPoseStrategy =
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR;
static const frc::AprilTagFieldLayout kTagLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo)};
static const Eigen::Matrix<double, 3, 1> kSingleTagStdDevs{4, 4, 8};
static const Eigen::Matrix<double, 3, 1> kMultiTagStdDevs{0.5, 0.5, 1};

const std::string kLimelightName = "limelight";
constexpr double kKnownPixelWidth = 58;
constexpr units::inch_t kNoteWidth = 14_in;
constexpr units::inch_t kKnownCalibrationDistance = 60_in;
constexpr units::inch_t kCalibrationDistanceAreaPercentage =
    kKnownCalibrationDistance * kKnownPixelWidth;
constexpr auto focalLength = kCalibrationDistanceAreaPercentage / kNoteWidth;

constexpr units::degree_t kCameraAngle = -20_deg;
constexpr units::inch_t kCameraLensHeight = 15_in;
constexpr double kConfidenceThreshold = 0.3;
constexpr double kTrigDistancePercentage = 0.5;
constexpr double kAreaPercentageThreshold = 0.04;

constexpr double kMinAngleDeg = -30.0;
constexpr double kMaxAngleDeg = 30.0;

static const wpi::array<double, 3> kDrivetrainStd = {0.1, 0.1, 0.1};
static const wpi::array<double, 3> kVisionStd = {0.9, 0.9, 0.9};

constexpr units::degree_t kGamepieceRotation = 180_deg;
constexpr frc::Pose2d kSimGamepiecePose =
    frc::Pose2d(7_m, 4_m, frc::Rotation2d(kGamepieceRotation));
}  // namespace VisionConstants