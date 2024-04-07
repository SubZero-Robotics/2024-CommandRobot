#include "utils/TurnToPose.h"

TurnToPose::TurnToPose(std::function<frc::Pose2d()> poseGetter)
    : m_poseGetter{poseGetter} {
  using namespace AutoConstants;

  // TODO: use constants
  auto xController = frc::PIDController(1, 0, 0);
  auto yController = frc::PIDController(1, 0, 0);
  auto profile = frc::TrapezoidProfile<units::radians>::Constraints(
      AutoConstants::kMaxAngularSpeed, AutoConstants::kMaxAngularAcceleration);
  auto profiledController = frc::ProfiledPIDController<units::radians>(
      kSnapToAngleP, kSnapToAngleI, kSnapToAngleD, profile);

  // TODO: use constants
  m_driveController = std::make_unique<frc::HolonomicDriveController>(
      xController, yController, profiledController);
  m_driveController->SetTolerance(
      frc::Pose2d(0.2_m, 0.2_m, frc::Rotation2d(0.5_deg)));
}

void TurnToPose::Update() {
  auto currentPose = m_poseGetter();
  m_speeds = m_driveController->Calculate(currentPose, m_targetPose, 0_mps,
                                          m_targetPose.Rotation());
}

void TurnToPose::SetTargetPose(frc::Pose2d pose) {
  m_startPose = m_poseGetter();
  m_targetPose = pose;
}

frc::ChassisSpeeds TurnToPose::GetSpeedCorrection() { return m_speeds; }

frc::ChassisSpeeds TurnToPose::BlendWithInput(const frc::ChassisSpeeds& other,
                                              double correctionFactor) {
  frc::ChassisSpeeds speeds{
      .vx = other.vx, .vy = other.vy, .omega = other.omega};

  speeds.omega =
      (speeds.omega * (1 - correctionFactor)) + (m_speeds.omega * correctionFactor);

  return speeds;
}