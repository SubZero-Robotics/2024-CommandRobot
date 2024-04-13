#include "subsystems/ScoringSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {}

void ScoringSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Amp upper velocity",
                                 m_ampUpperEnc.GetVelocity());
  frc::SmartDashboard::PutNumber("Amp lower velocity",
                                 m_ampLowerEnc.GetVelocity());
}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::SpinOutake() {
  m_ampUpperSpinnyBoi.Set(kScoringOutakeUpperSpeed);
  m_ampLowerSpinnyBoi.Set(kScoringOutakeLowerSpeed);
}

void ScoringSubsystem::Stop() {
  m_vectorSpinnyBoi.Set(0);
  m_speakerLowerSpinnyBoi.Set(0);
  m_speakerUpperSpinnyBoi.Set(0);
  m_ampLowerSpinnyBoi.Set(0);
  m_ampUpperSpinnyBoi.Set(0);
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
    SpinAmp(kAmpUpperSpeed, kAmpLowerSpeed);
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

void ScoringSubsystem::SpinAmp(double upperPercentage, double lowerPercentage) {
  m_ampLowerSpinnyBoi.Set(lowerPercentage);
  m_ampUpperSpinnyBoi.Set(upperPercentage);
  frc::SmartDashboard::PutNumber("Amp lower target %", lowerPercentage);
}

void ScoringSubsystem::SpinSpeaker() {
  m_speakerLowerSpinnyBoi.Set(-kSpeakerLowerSpeed);
  m_speakerUpperSpinnyBoi.Set(-kSpeakerUpperSpeed);
}

void ScoringSubsystem::SpinSubwoofer() {
  m_ampLowerSpinnyBoi.Set(kSubwooferLowerSpeed);
  m_ampUpperSpinnyBoi.Set(kSubwooferUpperSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  ConsoleLogger::getInstance().logVerbose("Amp Ramp Speed",
                                          m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) >= abs(MaxSpeedToRpm(kAmpLowerSpeed));
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Speaker Velocity %.3f",
                                          m_speakerLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Speaker Ramp Speed",
                                               m_speakerLowerEnc.GetVelocity());
  return abs(m_speakerLowerEnc.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSpeakerLowerSpeed));
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Subwoofer Velocity %.3f",
                                          m_ampLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Subwoofer Ramp Speed",
                                               m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}