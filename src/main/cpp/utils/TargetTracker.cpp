#include "utils/TargetTracker.h"

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

std::optional<DetectedObject> TargetTracker::GetBestTarget() {
  auto targets = GetTargets();

  if (targets.empty()) {
    return std::nullopt;
  }

  auto it =
      std::max_element(targets.begin(), targets.end(),
                       [](const DetectedObject& a, const DetectedObject& b) {
                         return a.confidence < b.confidence;
                       });
  return *it;
}

bool TargetTracker::HasTargetLock() {
  auto bestTarget = GetBestTarget();
  return bestTarget && bestTarget.value().confidence >= m_confidenceThreshold;
}

frc2::CommandPtr TargetTracker::MoveToIntakePose() {
  return frc2::DeferredCommand(
             [this] {
               auto targetPose = GetBestTargetPose();

               if (!targetPose) {
                 return frc2::InstantCommand([] {}).ToPtr();
               }

               return pathplanner::AutoBuilder::pathfindToPose(
                   targetPose.value(), kMovementConstraints,
                   0.0_mps,  // Goal end velocity in meters/sec
                   0.0_m  // Rotation delay distance in meters. This is how far
                          // the robot should travel before attempting to
                          // rotate.
               );
             },
             {})
      .ToPtr();
}

frc2::CommandPtr TargetTracker::IntakeTarget() {
  return frc2::DeferredCommand(
             [this] {
               bool hasTarget = HasTargetLock();

               if (!hasTarget) {
                 return frc2::InstantCommand([] {}).ToPtr();
               }

               return (MoveToIntakePose()
                           .AlongWith(
                               IntakingCommands::Intake(m_intake, m_scoring))
                           .WithTimeout(5_s))
                   .WithTimeout(20_s)
                   .FinallyDo([this] {
                     auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
                         0_mps, 0_mps, AutoConstants::kMaxAngularSpeed,
                         DriveConstants::kLoopTime);
                     m_drive->Drive(chassisSpeeds);
                   });
             },
             {})
      .ToPtr();
}

std::optional<frc::Pose2d> TargetTracker::GetBestTargetPose() {
  if (!HasTargetLock()) {
    return std::nullopt;
  }

  auto bestTarget = GetBestTarget().value();

  units::degree_t targetOffsetVertical = -bestTarget.centerY;
  units::degree_t verticalDelta = targetOffsetVertical - m_cameraAngle;
  units::inch_t heightDelta = -m_cameraHeight;
  units::radian_t verticalAngle = verticalDelta.convert<units::radian>();
  units::radian_t horizontalAngle = bestTarget.centerX.convert<units::radian>();
  units::inch_t distance = heightDelta / (tan(verticalAngle.value()));
  distance -= kDistanceGap;

  frc::Pose2d currentPose = m_drive->GetPose();
  frc::Rotation2d rotDelta = frc::Rotation2d(bestTarget.centerX);

  auto xTransformation = distance * cos(horizontalAngle.value());
  auto yTransformation = distance * sin(horizontalAngle.value());
  frc::Transform2d transformDelta =
      frc::Transform2d(xTransformation, yTransformation, rotDelta);

  return currentPose.TransformBy(transformDelta)
      .RotateBy(m_drive->GetHeading());
}