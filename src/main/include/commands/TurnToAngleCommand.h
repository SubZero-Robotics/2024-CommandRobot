#pragma once

// Based off
// https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/commands/TurnToAngle.java

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>
#include <memory>

#include "subsystems/DriveSubsystem.h"

class TurnToAngleCommand
    : public frc2::CommandHelper<frc2::Command, TurnToAngleCommand> {
 public:
  explicit TurnToAngleCommand(DriveSubsystem *drive,
                              std::function<units::degree_t()> angle,
                              bool relative = false)
      : isFinished{false},
        m_drive{drive},
        m_relative{relative},
        m_goalAngle{angle} {
    using namespace AutoConstants;
    using namespace TurnToPoseConstants;

    AddRequirements(drive);

    // Don't need constants for these since we're only rotating
    auto xController = frc::PIDController(1, 0, 0);
    auto yController = frc::PIDController(1, 0, 0);
    auto profile = frc::TrapezoidProfile<units::radians>::Constraints(
        AutoConstants::kMaxAngularSpeed,
        AutoConstants::kMaxAngularAcceleration);
    auto profiledController = frc::ProfiledPIDController<units::radians>(
        kTurnP, kTurnI, kTurnD, profile);
    m_driveController = std::make_unique<frc::HolonomicDriveController>(
        xController, yController, profiledController);
    m_driveController->SetTolerance(kPoseTolerance);
  }

  void Initialize() override {
    isFinished = false;
    m_startPose = m_drive->GetPose();
    if (m_relative) {
      m_targetPose = frc::Pose2d(
          m_startPose.Translation(),
          m_startPose.Rotation().RotateBy(frc::Rotation2d(m_goalAngle())));
    } else {
      m_targetPose = frc::Pose2d(m_startPose.Translation(),
                                 frc::Rotation2d(m_goalAngle()));
    }
  }

  void Execute() override {
    auto currentPose = m_drive->GetPose();
    auto chassisSpeeds = m_driveController->Calculate(
        currentPose, m_targetPose, 0_mps, m_targetPose.Rotation());
    m_drive->Drive(chassisSpeeds);
  }

  bool IsFinished() override { return m_driveController->AtReference(); }

  void End(bool interrupted) {
    frc::ChassisSpeeds chassisSpeeds = {
        .vx = 0_mps, .vy = 0_mps, .omega = 0_rad_per_s};
    m_drive->Drive(chassisSpeeds);
  }

 private:
  bool isFinished = false;
  DriveSubsystem *m_drive;

  bool m_relative;
  std::function<units::degree_t()> m_goalAngle;
  std::unique_ptr<frc::HolonomicDriveController> m_driveController;
  frc::Pose2d m_startPose;
  frc::Pose2d m_targetPose;
};