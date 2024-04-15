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
  return *it;
}

bool TargetTracker::HasTargetLock(std::vector<DetectedObject>& targets) {
  auto bestTarget = GetBestTarget(targets);
  return bestTarget && bestTarget.value().confidence >= m_confidenceThreshold;
}

frc2::CommandPtr TargetTracker::MoveToIntakePose() {
  return frc2::DeferredCommand(
             [this] {
               auto targets = GetTargets();
               auto targetPose = GetBestTargetPose(targets);

               if (!targetPose) {
                 ConsoleLogger::getInstance().logWarning("TargetTracker",
                                                         "NO TARGET FOUND");
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
               auto targets = GetTargets();
               bool hasTarget = HasTargetLock(targets);

               if (!hasTarget) {
                 ConsoleLogger::getInstance().logWarning("TargetTracker",
                                                         "NO TARGET FOUND");
                 return frc2::InstantCommand([] {}).ToPtr();
               }

               return (MoveToIntakePose()
                           .AlongWith(
                               IntakingCommands::Intake(m_intake, m_scoring))
                           .WithTimeout(10_s))
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

std::optional<frc::Pose2d> TargetTracker::GetBestTargetPose(
    std::vector<DetectedObject>& targets) {
  if (!frc::RobotBase::IsReal()) {
    return frc::Pose2d(7_m, 4_m, frc::Rotation2d(180_deg));
  }

  if (!HasTargetLock(targets)) {
    return std::nullopt;
  }

  auto bestTarget = GetBestTarget(targets).value();

  units::degree_t targetOffsetVertical = bestTarget.centerY;
  units::degree_t verticalDelta = targetOffsetVertical + m_cameraAngle;
  units::inch_t heightDelta = -m_cameraHeight;
  units::radian_t verticalAngle = verticalDelta.convert<units::radian>();
  units::radian_t horizontalAngle = bestTarget.centerX.convert<units::radian>();
  units::inch_t distance = heightDelta / tan(verticalAngle.value());

  frc::Pose2d currentPose = m_drive->GetPose();

  auto xTransformation = distance * tan(horizontalAngle.value());
  auto yTransformation = distance;
  auto hypotDistance =
      units::inch_t(pow(cos(horizontalAngle.value()) / distance.value(), -1));

  frc::Transform2d transformDelta =
      frc::Transform2d(-xTransformation, yTransformation, 0_deg);
  frc::Pose2d notePose = currentPose.TransformBy(transformDelta);

  // Do it the WPI way - currently just used for testing
  auto wpiTranslation = frc::Translation2d(
      hypotDistance.convert<units::meter>(), frc::Rotation2d(horizontalAngle));
  auto wpiTransformation =
      frc::Transform2d(wpiTranslation, currentPose.Rotation());
  auto wpiFinalPose = currentPose.TransformBy(wpiTransformation);

  // // Pivot around our robot's current pose to be field-oriented
  // double s = sin(currentPose.Rotation().Radians().value());
  // double c = cos(currentPose.Rotation().Radians().value());

  // auto originX = notePose.X() - currentPose.X();
  // auto originY = notePose.Y() - currentPose.Y();

  // auto newX = (originX * c) - (originY * s);
  // auto newY = (originX * s) + (originY * c);

  // return frc::Pose2d(newX + currentPose.X(), newY + currentPose.Y(), 0_deg);

  return wpiFinalPose;
}