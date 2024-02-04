#include "subsystems/ScoringSubsystem.h"

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {
  m_speakerLowerSpinnyBoi.Follow(m_speakerUpperSpinnyBoi, true);

  m_speakerPidController.SetP(SpeakerPID::kP);
  m_speakerPidController.SetI(SpeakerPID::kI);
  m_speakerPidController.SetD(SpeakerPID::kD);
  m_speakerPidController.SetIZone(SpeakerPID::kIZone);
  m_speakerPidController.SetFF(SpeakerPID::kFF);

  m_ampUpperPidController.SetP(AmpUpperPID::kP);
  m_ampUpperPidController.SetI(AmpUpperPID::kI);
  m_ampUpperPidController.SetD(AmpUpperPID::kD);
  m_ampUpperPidController.SetIZone(AmpUpperPID::kIZone);
  m_ampUpperPidController.SetFF(AmpUpperPID::kFF);

  m_ampLowerPidController.SetP(AmpLowerPID::kP);
  m_ampLowerPidController.SetI(AmpLowerPID::kI);
  m_ampLowerPidController.SetD(AmpLowerPID::kD);
  m_ampLowerPidController.SetIZone(AmpLowerPID::kIZone);
  m_ampLowerPidController.SetFF(AmpLowerPID::kFF);
}

void ScoringSubsystem::Periodic() {}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::Stop() {
  m_vectorSpinnyBoi.Set(0);
  m_ampLowerSpinnyBoi.Set(0);
  m_ampUpperSpinnyBoi.Set(0);
  m_speakerUpperSpinnyBoi.Set(0);
}

void ScoringSubsystem::SpinVectorSide(ScoringDirection direction) {
  if (direction == ScoringDirection::AmpSide ||
      direction == ScoringDirection::Subwoofer) {
    m_vectorSpinnyBoi.Set(ScoringConstants::kVectorSpeed);
    return;
  }

  m_vectorSpinnyBoi.Set(-ScoringConstants::kVectorSpeed);
}

void ScoringSubsystem::StartScoringRamp(ScoringDirection direction) {
  if (direction == ScoringDirection::AmpSide) {
    SpinAmp();
    return;
  }

  if (direction == ScoringDirection::Subwoofer) {
    SpinSubwoofer();
    return;
  }

  SpinSpeaker();
}

bool ScoringSubsystem::GetMotorAtScoringSpeed(ScoringDirection direction) {
  if (direction == ScoringDirection::AmpSide) {
    return CheckAmpSpeed();
  }

  if (direction == ScoringDirection::Subwoofer) {
    return CheckSubwooferSpeed();
  }

  return CheckSpeakerSpeed();
}

bool ScoringSubsystem::GetMotorFreeWheel(ScoringDirection direction) {
  if (direction == ScoringDirection::AmpSide) {
    return m_ampLowerSpinnyBoi.GetOutputCurrent() <
           ScoringConstants::kFreeSpinCurrentThreshold;
  }

  if (direction == ScoringDirection::Subwoofer) {
    return m_speakerUpperSpinnyBoi.GetOutputCurrent() <
           ScoringConstants::kFreeSpinCurrentThreshold;
  }

  return m_speakerLowerSpinnyBoi.GetOutputCurrent() <
         ScoringConstants::kFreeSpinCurrentThreshold;
}

void ScoringSubsystem::SpinAmp() {
  m_ampLowerPidController.SetReference(
      ScoringConstants::kAmpLowerSpeed,
      rev::CANSparkMax::ControlType::kVelocity);
  m_ampUpperPidController.SetReference(
      ScoringConstants::kAmpUpperSpeed,
      rev::CANSparkMax::ControlType::kVelocity);
}

void ScoringSubsystem::SpinSpeaker() {
  m_speakerPidController.SetReference(ScoringConstants::kSpeakerUpperSpeed,
                                      rev::CANSparkMax::ControlType::kVelocity);
}

void ScoringSubsystem::SpinSubwoofer() {
  m_ampLowerPidController.SetReference(
      ScoringConstants::kSubwooferLowerSpeed,
      rev::CANSparkMax::ControlType::kVelocity);
  m_ampUpperPidController.SetReference(
      ScoringConstants::kSubwooferUpperSpeed,
      rev::CANSparkMax::ControlType::kVelocity);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  // TODO: Check range rather than exact equals
  return m_ampEncoder.GetVelocity() >= ScoringConstants::kAmpLowerSpeed;
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  // TODO: Check range rather than exact equals
  return abs(m_speakerEncoder.GetVelocity()) >=
         abs(ScoringConstants::kSpeakerUpperSpeed);
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  // TODO: Check range rather than exact equals
  return abs(m_ampEncoder.GetVelocity()) >=
         abs(ScoringConstants::kSubwooferLowerSpeed);
}
