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

class TurnToAngle : public ITurnToTarget {
 public:
  struct TurnToAngleConfig {
    frc::TrapezoidProfile<units::radians>::Constraints rotationConstraints;
    double turnP;
    double turnI;
    double turnD;
    units::degree_t tolerance;
  };

  explicit TurnToAngle(TurnToAngleConfig config,
                       std::function<frc::Pose2d()> poseGetter,
                       std::function<frc::Field2d*()> fieldGetter);

  void Update() override;

  void SetTargetAngleAbsolute(units::degree_t angle);

  void SetTargetAngleRelative(units::degree_t angle);

  frc::ChassisSpeeds GetSpeedCorrection() override;

  /**
   * @param other The initial ChassisSpeeds
   * @param correctionFactor Ranges from 0 to 1; percentage of the TurnToPose
   * correction to apply
   */
  frc::ChassisSpeeds BlendWithInput(const frc::ChassisSpeeds& other,
                                    double correctionFactor) override;

  inline bool AtGoal() override { return m_driveController->AtReference(); }

  inline std::optional<units::degree_t> GetTargetAngle() const {
    return m_targetAngle;
  }

 private:
  TurnToAngleConfig m_config;
  std::function<frc::Pose2d()> m_poseGetter;
  std::function<frc::Field2d*()> m_fieldGetter;
  std::unique_ptr<frc::HolonomicDriveController> m_driveController;
  units::degree_t m_startAngle;
  std::optional<units::degree_t> m_targetAngle;
  frc::ChassisSpeeds m_speeds;
};