#pragma once

#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

const pathplanner::PathConstraints kMovementConstraints{
    3.0_mps, 1.5_mps_sq, 540_deg_per_s, 720_deg_per_s_sq};
const std::string kLimelightName = "limelight";
// The distance to stop short of the target
constexpr units::inch_t kDistanceGap = 0_in;

struct DetectedObject {
  uint8_t classId;
  double confidence;
  // Positive-right, center-zero
  units::degree_t centerX;
  // Positive-down, center-zero
  units::degree_t centerY;

  DetectedObject(uint8_t id, double conf, units::degree_t cX,
                 units::degree_t cY)
      : classId{id}, confidence{conf}, centerX{cX}, centerY{cY} {}

  DetectedObject(const LimelightHelpers::DetectionResultClass& detectionResult)
      : classId{(uint8_t)detectionResult.m_classID},
        confidence{detectionResult.m_confidence},
        centerX{detectionResult.m_TargetXDegreesCrosshairAdjusted},
        centerY{detectionResult.m_TargetYDegreesCrosshairAdjusted} {}
};

class TargetTracker {
 public:
  TargetTracker(units::degree_t cameraAngle, units::inch_t cameraLensHeight,
                double confidenceThreshold, IntakeSubsystem* intake,
                ScoringSubsystem* scoring, DriveSubsystem* drive);
  std::vector<DetectedObject> GetTargets();
  std::optional<DetectedObject> GetBestTarget();
  bool HasTargetLock();
  frc2::CommandPtr MoveToIntakePose();
  frc2::CommandPtr IntakeTarget();
  std::optional<frc::Pose2d> GetBestTargetPose();

 private:
  units::degree_t m_cameraAngle;
  units::inch_t m_cameraHeight;
  double m_confidenceThreshold;
  IntakeSubsystem* m_intake;
  ScoringSubsystem* m_scoring;
  DriveSubsystem* m_drive;
};