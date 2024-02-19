#include "subsystems/ScoringSubsystem.h"

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {
  // m_speakerLowerSpinnyBoi.Follow(m_speakerUpperSpinnyBoi, false);

  // m_speakerPidController.SetP(SpeakerPID::kP);
  // m_speakerPidController.SetI(SpeakerPID::kI);
  // m_speakerPidController.SetD(SpeakerPID::kD);
  // m_speakerPidController.SetIZone(SpeakerPID::kIZone);
  // m_speakerPidController.SetFF(SpeakerPID::kFF);

  // m_ampUpperPidController.SetP(AmpUpperPID::kP);
  // m_ampUpperPidController.SetI(AmpUpperPID::kI);
  // m_ampUpperPidController.SetD(AmpUpperPID::kD);
  // m_ampUpperPidController.SetIZone(AmpUpperPID::kIZone);
  // m_ampUpperPidController.SetFF(AmpUpperPID::kFF);

  // m_ampLowerPidController.SetP(AmpLowerPID::kP);
  // m_ampLowerPidController.SetI(AmpLowerPID::kI);
  // m_ampLowerPidController.SetD(AmpLowerPID::kD);
  // m_ampLowerPidController.SetIZone(AmpLowerPID::kIZone);
  // m_ampLowerPidController.SetFF(AmpLowerPID::kFF);
}

void ScoringSubsystem::Periodic() {}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::SpinOuttake() {
  m_ampLowerSpinnyBoi.Set(-ScoringConstants::kScoringOutakeSpeed);
  m_ampUpperSpinnyBoi.Set(ScoringConstants::kScoringOutakeSpeed);
  m_speakerUpperSpinnyBoi.Set(ScoringConstants::kScoringOutakeSpeed);
  m_speakerLowerSpinnyBoi.Set(-ScoringConstants::kScoringOutakeSpeed);
};

void ScoringSubsystem::Stop() {
  m_vectorSpinnyBoi.Set(0);
  m_ampLowerSpinnyBoi.Set(0);
  m_ampUpperSpinnyBoi.Set(0);
  m_speakerUpperSpinnyBoi.Set(0);
  m_speakerLowerSpinnyBoi.Set(0);
}

void ScoringSubsystem::SpinVectorSide(ScoringDirection direction) {
  if (direction == ScoringDirection::AmpSide ||
      direction == ScoringDirection::Subwoofer) {
    m_vectorSpinnyBoi.Set(kVectorSpeed);
    return;
  }

  m_vectorSpinnyBoi.Set(-kVectorSpeed);
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
    ConsoleLogger::getInstance().logVerbose(
        "Amp free wheel", "Current: %.3f",
        m_ampLowerSpinnyBoi.GetOutputCurrent());
    return m_ampLowerSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
  }

  if (direction == ScoringDirection::Subwoofer) {
    ConsoleLogger::getInstance().logVerbose(
        "Subwoofer free wheel", "Current: %.3f",
        m_ampUpperSpinnyBoi.GetOutputCurrent());
    return m_ampUpperSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
  }

  ConsoleLogger::getInstance().logVerbose(
      "Other free wheel", "Current: %.3f",
      m_ampUpperSpinnyBoi.GetOutputCurrent());
  return m_speakerLowerSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
}

void ScoringSubsystem::SpinAmp() {
  // m_ampLowerPidController.SetReference(
  //     kAmpLowerSpeed,
  //     rev::CANSparkMax::ControlType::kVelocity);
  // m_ampUpperPidController.SetReference(
  //     kAmpUpperSpeed,
  //     rev::CANSparkMax::ControlType::kVelocity);
  m_ampLowerSpinnyBoi.Set(kAmpLowerSpeed);
  m_ampUpperSpinnyBoi.Set(kAmpUpperSpeed);
}

void ScoringSubsystem::SpinSpeaker() {
  // m_speakerPidController.SetReference(kSpeakerUpperSpeed,
  //                                     rev::CANSparkMax::ControlType::kVelocity);
  m_speakerUpperSpinnyBoi.Set(kSpeakerUpperSpeed);
  m_speakerLowerSpinnyBoi.Set(kSpeakerLowerSpeed);
}

void ScoringSubsystem::SpinSubwoofer() {
  // m_ampLowerPidController.SetReference(
  //     kSubwooferLowerSpeed,
  //     rev::CANSparkMax::ControlType::kVelocity);
  // m_ampUpperPidController.SetReference(
  //     kSubwooferUpperSpeed,
  //     rev::CANSparkMax::ControlType::kVelocity);
  m_ampLowerSpinnyBoi.Set(kSubwooferLowerSpeed);
  m_ampUpperSpinnyBoi.Set(kSubwooferUpperSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose(
      "Scoring Subsystem", "Amp Velocity %.3f", m_ampEncoder.GetVelocity());
  return abs(m_ampEncoder.GetVelocity()) - 750 >=
         abs(MaxSpeedToRpm(kAmpLowerSpeed));
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Speaker Velocity %.3f",
                                          m_speakerEncoder.GetVelocity());
  return abs(m_speakerEncoder.GetVelocity()) + 1000 >=
         abs(MaxSpeedToRpm(kSpeakerLowerSpeed));
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Subwoofer Velocity %.3f",
                                          m_ampEncoder.GetVelocity());
  return abs(m_ampEncoder.GetVelocity()) + 1500 >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}
