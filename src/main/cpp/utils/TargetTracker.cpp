#include "utils/TargetTracker.h"

#include <frc/RobotBase.h>
#include <frc2/command/DeferredCommand.h>

#include "commands/DriveVelocityCommand.h"
#include "utils/Commands/IntakeCommands.h"

TargetTracker::TargetTracker(units::degree_t cameraAngle,
                             units::inch_t cameraLensHeight,
                             double confidenceThreshold,
                             IntakeSubsystem* intake, ScoringSubsystem* scoring,
                             DriveSubsystem* drive)
    : m_cameraAngle{cameraAngle},
      m_cameraHeight{cameraLensHeight},
      m_confidenceThreshold{confidenceThreshold},
      m_intake{intake},
      m_scoring{scoring},
      m_drive{drive} {}

std::vector<DetectedObject> TargetTracker::GetTargets() {
  auto llResult = LimelightHelpers::getLatestResults(kLimelightName);
  auto& detectionResults = llResult.targetingResults.DetectionResults;

  std::vector<DetectedObject> objects;
  objects.reserve(detectionResults.size());

  std::transform(detectionResults.begin(), detectionResults.end(),
                 std::back_inserter(objects),
                 [](const LimelightHelpers::DetectionResultClass& det) {
                   return DetectedObject(det);
                 });

  return objects;
}

std::optional<DetectedObject> TargetTracker::GetBestTarget(
    std::vector<DetectedObject>& targets) {
  if (targets.empty()) {
    return std::nullopt;
  }

  auto it =
      std::max_element(targets.begin(), targets.end(),
                       [](const DetectedObject& a, const DetectedObject& b) {
                         return a.confidence < b.confidence;
                       });

  auto corners = LimelightHelpers::getCurrentCorners(kLimelightName);
  if (corners) {
    it->withRawCorners(corners.value());
  }

  return *it;
}

bool TargetTracker::HasTargetLock(std::vector<DetectedObject>& targets) {
  auto bestTarget = GetBestTarget(targets);
  return bestTarget && bestTarget.value().confidence >= m_confidenceThreshold;
}

std::optional<frc::Pose2d> TargetTracker::GetBestTargetPose(
    std::vector<DetectedObject>& targets) {
  if (!frc::RobotBase::IsReal()) {
    return frc::Pose2d(7_m, 4_m, frc::Rotation2d(180_deg));
  }

  if (!HasTargetLock(targets)) {
    return std::nullopt;
  }

  auto bestTarget = GetBestTarget(targets).value();

  return GetTargetPose(bestTarget);
}

std::optional<frc::Pose2d> TargetTracker::GetTargetPose(
    const DetectedObject& object) {
  units::radian_t horizontalAngle = object.centerX.convert<units::radian>();

  frc::Pose2d currentPose = m_drive->GetPose();

  auto distance = GetDistanceToTarget(object);

  auto xTransformation = distance * tan(horizontalAngle.value());
  auto yTransformation = distance;
  auto hypotDistance =
      units::inch_t(pow(cos(horizontalAngle.value()) / distance.value(), -1));

  frc::Transform2d transformDelta =
      frc::Transform2d(xTransformation, yTransformation, 0_deg);

  // Do it the WPI way
  auto wpiTranslation = frc::Translation2d(
      hypotDistance.convert<units::meter>(), -frc::Rotation2d(horizontalAngle));
  auto wpiTransformation =
      frc::Transform2d(wpiTranslation, currentPose.Rotation());
  auto wpiFinalPose = currentPose.TransformBy(wpiTransformation);

  return frc::Pose2d(wpiFinalPose.Translation(), 180_deg);
}

units::inch_t TargetTracker::GetDistanceToTarget(const DetectedObject& target) {
  units::degree_t targetOffsetVertical = target.centerY;
  units::degree_t verticalDelta = targetOffsetVertical + m_cameraAngle;
  units::radian_t verticalAngle = verticalDelta.convert<units::radian>();

  DetectedCorners corners = target.detectedCorners;
  double pixelWidth = corners.bottomRight.x - corners.bottomLeft.x;
  frc::SmartDashboard::PutNumber("Pixel Width", pixelWidth);

  auto otherDistance =
      (VisionConstants::kNoteWidth * VisionConstants::focalLength) / pixelWidth;

  units::inch_t heightDelta = -m_cameraHeight;
  units::inch_t distance = heightDelta / tan(verticalAngle.value());
  frc::SmartDashboard::PutString("TargetTracker otherDistance",
                                 std::to_string(otherDistance.value()) + " in");

  auto combinedDistance = (distance * 0.5) + (otherDistance * 0.5);
  frc::SmartDashboard::PutString("TargetTracker combinedDistance",
                                 std::to_string(otherDistance.value()) + " in");

  return combinedDistance;
}