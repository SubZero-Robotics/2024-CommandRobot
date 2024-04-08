#include "utils/TurnToPose.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

TurnToPose::TurnToPose(std::function<frc::Pose2d()> poseGetter,
                       std::function<frc::Field2d*()> fieldGetter)
    : m_poseGetter{poseGetter}, m_fieldGetter{fieldGetter} {
  using namespace AutoConstants;

  // TODO: use constants
  auto xController = frc::PIDController(1, 0, 0);
  auto yController = frc::PIDController(1, 0, 0);
  auto profile = frc::TrapezoidProfile<units::radians>::Constraints(
      960_deg_per_s, 1200_deg_per_s_sq);
  auto profiledController =
      frc::ProfiledPIDController<units::radians>(5, 0, kSnapToAngleD, profile);

  // TODO: use constants
  m_driveController = std::make_unique<frc::HolonomicDriveController>(
      xController, yController, profiledController);
  m_driveController->SetTolerance(
      frc::Pose2d(0.2_m, 0.2_m, frc::Rotation2d(0.5_deg)));
}

void TurnToPose::Update() {
  auto currentPose = m_poseGetter();
  auto diff = currentPose.Translation() - m_targetPose.Translation();

  auto newDegree = units::radian_t(atan2(diff.Y().value(), diff.X().value()))
                       .convert<units::degree>();

  frc::SmartDashboard::PutNumber("newDegree deg", newDegree.value());

  auto angleOffset = m_targetPose.Rotation().Degrees() + newDegree;

  frc::SmartDashboard::PutNumber("target deg", angleOffset.value());

  frc::Pose2d newTargetPose(m_targetPose.Translation(),
                            frc::Rotation2d(angleOffset));

  auto* field = m_fieldGetter();
  field->GetObject("pose_target")->SetPose(m_targetPose);

  m_speeds = m_driveController->Calculate(currentPose, newTargetPose, 0_mps,
                                          newTargetPose.Rotation());
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

  speeds.omega = (speeds.omega * (1 - correctionFactor)) +
                 (m_speeds.omega * correctionFactor);

  return speeds;
}