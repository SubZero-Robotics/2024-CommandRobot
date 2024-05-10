#pragma once

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>

#include <functional>
#include <memory>

#include "utils/ITurnToTarget.h"

class TurnToPose : public ITurnToTarget {
 public:
  struct TurnToPoseConfig {
    frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints;
    double turnP;
    double turnI;
    double turnD;
    /// @brief A pose within this range will be considered at-goal
    frc::Pose2d poseTolerance;
  };

  explicit TurnToPose(TurnToPoseConfig config,
                      std::function<frc::Pose2d()> poseGetter,
                      std::function<frc::Field2d*()> fieldGetter);

  void Update() override;

  void SetTargetPose(frc::Pose2d pose);

  void SetTargetAngleRelative(units::degree_t angle);

  void SetTargetAngleAbsolute(units::degree_t angle);

  frc::ChassisSpeeds GetSpeedCorrection() override;
  /**
   * @param currentPose
   * @param targetPose
   */
  static units::degree_t GetAngleFromOtherPose(const frc::Pose2d&,
                                               const frc::Pose2d&);

  /**
   * @param other The initial ChassisSpeeds
   * @param correctionFactor Ranges from 0 to 1; percentage of the TurnToPose
   * correction to apply
   */
  frc::ChassisSpeeds BlendWithInput(const frc::ChassisSpeeds& other,
                                    double correctionFactor) override;

  inline bool AtGoal() override { return m_driveController->AtReference(); }

  inline std::optional<frc::Pose2d> GetTargetPose() const {
    return m_targetPose;
  }

  inline std::optional<units::degree_t> GetTargetAngle() const {
    return m_targetAngle;
  }

  inline units::degree_t GetTargetHeading() const { return m_targetHeading; }

  static double NormalizeScalar(double x, double from_min, double from_max, double to_min, double to_max) {
    return (x - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
  }

 private:
  TurnToPoseConfig m_config;
  std::function<frc::Pose2d()> m_poseGetter;
  std::function<frc::Field2d*()> m_fieldGetter;
  std::unique_ptr<frc::HolonomicDriveController> m_driveController;
  frc::Pose2d m_startPose;
  std::optional<frc::Pose2d> m_targetPose;
  std::optional<units::degree_t> m_targetAngle;
  units::degree_t m_targetHeading;
  frc::ChassisSpeeds m_speeds;
};