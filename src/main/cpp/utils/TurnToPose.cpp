#include "utils/TurnToPose.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

TurnToPose::TurnToPose(TurnToPoseConfig config,
                       std::function<frc::Pose2d()> poseGetter,
                       std::function<frc::Field2d*()> fieldGetter)
    : m_config{config}, m_poseGetter{poseGetter}, m_fieldGetter{fieldGetter} {
  using namespace AutoConstants;

  auto xController = frc::PIDController(TurnToPoseConstants::kTurnTranslationP,
                                        TurnToPoseConstants::kTurnTranslationI,
                                        TurnToPoseConstants::kTurnTranslationD);
  auto yController = frc::PIDController(TurnToPoseConstants::kTurnTranslationP,
                                        TurnToPoseConstants::kTurnTranslationI,
                                        TurnToPoseConstants::kTurnTranslationD);
  auto profile = m_config.rotationConstraints;
  auto profiledController = frc::ProfiledPIDController<units::radians>(
      m_config.turnP, m_config.turnI, m_config.turnD, profile);

  m_driveController = std::make_unique<frc::HolonomicDriveController>(
      xController, yController, profiledController);
  m_driveController->SetTolerance(m_config.poseTolerance);
}

void TurnToPose::Update() {
  if (!m_targetPose) return;

  auto currentPose = m_poseGetter();
  auto angleOffset = GetAngleFromOtherPose(currentPose, m_targetPose.value());
  frc::SmartDashboard::PutNumber("Angle offset norm", angleOffset.value());
  m_targetHeading = angleOffset;
  frc::Pose2d newTargetPose(m_targetPose.value().Translation(),
                            frc::Rotation2d(angleOffset));

  auto* field = m_fieldGetter();
  field->GetObject("pose_target")->SetPose(m_targetPose.value());

  m_speeds = m_driveController->Calculate(currentPose, newTargetPose, 0_mps,
                                          newTargetPose.Rotation());
}

units::degree_t TurnToPose::GetAngleFromOtherPose(
    const frc::Pose2d& currentPose, const frc::Pose2d& otherPose) {
  auto diff = currentPose.Translation() - otherPose.Translation();

  auto newDegree = units::radian_t(atan2(diff.Y().value(), diff.X().value()))
                       .convert<units::degree>();
  return otherPose.Rotation().Degrees() + newDegree;
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