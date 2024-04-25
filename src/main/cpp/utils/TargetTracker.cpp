#include "utils/TargetTracker.h"

#include <frc/RobotBase.h>
#include <frc2/command/DeferredCommand.h>

#include <ranges>

#include "commands/DriveVelocityCommand.h"
#include "utils/Commands/IntakeCommands.h"

TargetTracker::TargetTracker(TargetTrackerConfig config,
                             IntakeSubsystem* intake, ScoringSubsystem* scoring,
                             DriveSubsystem* drive)
    : m_config{config}, m_intake{intake}, m_scoring{scoring}, m_drive{drive} {}

std::vector<DetectedObject> TargetTracker::GetTargets() {
  auto llResult = LimelightHelpers::getLatestResults(m_config.limelightName);
  auto& detectionResults = llResult.targetingResults.DetectionResults;

  std::vector<DetectedObject> objects;
  objects.reserve(detectionResults.size());

  std::transform(detectionResults.begin(), detectionResults.end(),
                 std::back_inserter(objects),
                 [](const LimelightHelpers::DetectionResultClass& det) {
                   return DetectedObject(det);
                 });

  std::vector<DetectedObject> filteredObjects;
  std::copy_if(objects.begin(), objects.end(),
               std::inserter(filteredObjects, filteredObjects.end()),
               [this](const DetectedObject& object) {
                 return object.areaPercentage >=
                        m_config.areaPercentageThreshold;
               });

  return objects;
}

std::optional<DetectedObject> TargetTracker::GetBestTarget(
    std::vector<DetectedObject>& targets) {
  if (!frc::RobotBase::IsReal()) {
    return DetectedObject(1, 0.75, 20_deg, 30_deg, 0.08, {});
  }

  if (targets.empty()) {
    return std::nullopt;
  }

  auto it =
      std::max_element(targets.begin(), targets.end(),
                       [](const DetectedObject& a, const DetectedObject& b) {
                         return a.confidence < b.confidence;
                       });

  auto corners = LimelightHelpers::getCurrentCorners(m_config.limelightName);
  if (corners) {
    it->withRawCorners(corners.value());
  }

  return *it;
}

bool TargetTracker::HasTargetLock(std::vector<DetectedObject>& targets) {
  auto bestTarget = GetBestTarget(targets);
  return bestTarget &&
         bestTarget.value().confidence >= m_config.confidenceThreshold;
}

std::optional<frc::Pose2d> TargetTracker::GetBestTargetPose(
    std::vector<DetectedObject>& targets) {
  if (!frc::RobotBase::IsReal()) {
    return m_config.simGamepiecePose;
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

  return frc::Pose2d(wpiFinalPose.Translation(), m_config.gamepieceRotation);
}

units::inch_t TargetTracker::GetDistanceToTarget(const DetectedObject& target) {
  units::degree_t targetOffsetVertical = target.centerY;
  units::degree_t verticalDelta = targetOffsetVertical + m_config.cameraAngle;
  units::radian_t verticalAngle = verticalDelta.convert<units::radian>();

  DetectedCorners corners = target.detectedCorners;
  double pixelWidth = corners.bottomRight.x - corners.bottomLeft.x;
  frc::SmartDashboard::PutNumber("TargetTracker Pixel Width", pixelWidth);

  auto otherDistance =
      pixelWidth > 0
          ? ((m_config.gamepieceWidth * m_config.focalLength) / pixelWidth)
          : 0_in;

  units::inch_t heightDelta = -m_config.cameraLensHeight;
  units::inch_t distance = heightDelta / tan(verticalAngle.value());
  frc::SmartDashboard::PutString("TargetTracker otherDistance",
                                 std::to_string(otherDistance.value()) + " in");

  auto combinedDistance =
      (distance * m_config.trigDistancePercentage) +
      (otherDistance * (1 - m_config.trigDistancePercentage));
  frc::SmartDashboard::PutString("TargetTracker combinedDistance",
                                 std::to_string(otherDistance.value()) + " in");

  return combinedDistance;
}