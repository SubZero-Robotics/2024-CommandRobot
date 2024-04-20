#pragma once

#include <pathplanner/lib/commands/FollowPathHolonomic.h>

#include <string>
#include <vector>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ScoringSubsystem.h"

struct DetectedCorner {
  double x;
  double y;

  DetectedCorner() {}

  explicit DetectedCorner(const std::vector<double>& coord) {
    x = coord[0];
    y = coord[1];
  }
};

struct DetectedCorners {
  DetectedCorner topLeft;
  DetectedCorner topRight;
  DetectedCorner bottomLeft;
  DetectedCorner bottomRight;

  DetectedCorners() {}

  explicit DetectedCorners(const std::vector<std::vector<double>>& corners) {
    if (corners.empty()) return;

    topLeft = DetectedCorner(corners[0]);
    topRight = DetectedCorner(corners[1]);
    bottomLeft = DetectedCorner(corners[2]);
    bottomRight = DetectedCorner(corners[3]);
  }

  explicit DetectedCorners(const std::vector<double>& rawCorners) {
    if (rawCorners.size() < 8) return;

    topLeft = DetectedCorner({rawCorners[0], rawCorners[1]});
    bottomLeft = DetectedCorner({rawCorners[6], rawCorners[7]});
    bottomRight = DetectedCorner({rawCorners[4], rawCorners[5]});
    topRight = DetectedCorner({rawCorners[2], rawCorners[3]});
  }
};

struct DetectedObject {
  uint8_t classId;
  double confidence;
  // Positive-right, center-zero
  units::degree_t centerX;
  // Positive-down, center-zero
  units::degree_t centerY;
  double areaPercentage;
  DetectedCorners detectedCorners;

  explicit DetectedObject(uint8_t id, double conf, units::degree_t cX,
                          units::degree_t cY, double area,
                          std::vector<std::vector<double>> corners)
      : classId{id},
        confidence{conf},
        centerX{cX},
        centerY{cY},
        areaPercentage{area},
        detectedCorners{corners} {}

  explicit DetectedObject(
      const LimelightHelpers::DetectionResultClass& detectionResult)
      : classId{static_cast<uint8_t>(detectionResult.m_classID)},
        confidence{detectionResult.m_confidence},
        centerX{detectionResult.m_TargetXDegreesCrosshairAdjusted},
        centerY{detectionResult.m_TargetYDegreesCrosshairAdjusted},
        areaPercentage{detectionResult.m_TargetAreaNormalized},
        detectedCorners{detectionResult.m_TargetCorners} {}

  void withRawCorners(const std::vector<double>& rawCorners) {
    detectedCorners = DetectedCorners(rawCorners);
  }
};

class TargetTracker {
 public:
  struct TargetTrackerConfig {
    units::degree_t cameraAngle;
    units::inch_t cameraLensHeight;
    double confidenceThreshold;
    std::string limelightName;
    units::inch_t gamepieceWidth;
    units::dimensionless::scalar_t focalLength;
    frc::Pose2d simGamepiecePose;
    units::degree_t gamepieceRotation;
    /// @brief Ranges from 0 to 1; Multiplies trig-based distances and then
    /// applies the inverse to width-based estimate
    double trigDistancePercentage;
  };

  TargetTracker(TargetTrackerConfig config, IntakeSubsystem* intake,
                ScoringSubsystem* scoring, DriveSubsystem* drive);
  std::vector<DetectedObject> GetTargets();
  std::optional<DetectedObject> GetBestTarget(std::vector<DetectedObject>&);
  bool HasTargetLock(std::vector<DetectedObject>&);
  std::optional<frc::Pose2d> GetTargetPose(const DetectedObject&);
  std::optional<frc::Pose2d> GetBestTargetPose(std::vector<DetectedObject>&);
  units::inch_t GetDistanceToTarget(const DetectedObject&);

 private:
  TargetTrackerConfig m_config;
  IntakeSubsystem* m_intake;
  ScoringSubsystem* m_scoring;
  DriveSubsystem* m_drive;
};