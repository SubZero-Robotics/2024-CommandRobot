#include "utils/TurnToAngle.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

TurnToAngle::TurnToAngle(TurnToAngleConfig config,
                         std::function<frc::Pose2d()> poseGetter,
                         std::function<frc::Field2d*()> fieldGetter)
    : m_config{config}, m_poseGetter{poseGetter}, m_fieldGetter{fieldGetter} {
  using namespace AutoConstants;
  using namespace TurnToPoseConstants;

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
  m_driveController->SetTolerance(kPoseTolerance);
}

void TurnToAngle::Update() {
  if (!m_targetAngle) return;

  auto currentPose = m_poseGetter();
  frc::SmartDashboard::PutNumber("Target angle deg",
                                 m_targetAngle.value().value());
  frc::Pose2d newTargetPose(currentPose.Translation(),
                            frc::Rotation2d(m_targetAngle.value()));

  auto* field = m_fieldGetter();
  field->GetObject("rotation pose_target")->SetPose(newTargetPose);

  m_speeds = m_driveController->Calculate(currentPose, newTargetPose, 0_mps,
                                          newTargetPose.Rotation());
}

void TurnToAngle::SetTargetAngleAbsolute(units::degree_t angle) {
  m_targetAngle = angle;
}

void TurnToAngle::SetTargetAngleRelative(units::degree_t angle) {
  m_targetAngle = m_poseGetter().RotateBy(angle).Rotation().Degrees();
}

frc::ChassisSpeeds TurnToAngle::GetSpeedCorrection() { return m_speeds; }

frc::ChassisSpeeds TurnToAngle::BlendWithInput(const frc::ChassisSpeeds& other,
                                               double correctionFactor) {
  frc::ChassisSpeeds speeds{
      .vx = other.vx, .vy = other.vy, .omega = other.omega};

  speeds.omega = (speeds.omega * (1 - correctionFactor)) +
                 (m_speeds.omega * correctionFactor);

  return speeds;
}
