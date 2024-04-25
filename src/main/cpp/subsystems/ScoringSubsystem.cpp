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

void ScoringSubsystem::SpinOutake(double upperPercentage,
                                  double lowerPercentage) {
  m_ampUpperSpinnyBoi.Set(upperPercentage);
  m_ampLowerSpinnyBoi.Set(lowerPercentage);
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

  if (direction == ScoringDirection::FeedPodium) {
    SpinSpeaker(kFeedUpperSpeed, kFeedLowerSpeed);
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
    ConsoleWriter.logVerbose("Amp free wheel", "Current: %.3f",
                             m_ampLowerSpinnyBoi.GetOutputCurrent());
    return m_ampLowerSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
  }

  if (direction == ScoringDirection::Subwoofer) {
    ConsoleWriter.logVerbose("Subwoofer free wheel", "Current: %.3f",
                             m_ampUpperSpinnyBoi.GetOutputCurrent());
    return m_ampUpperSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
  }

  ConsoleWriter.logVerbose("Other free wheel", "Current: %.3f",
                           m_ampUpperSpinnyBoi.GetOutputCurrent());
  return m_speakerLowerSpinnyBoi.GetOutputCurrent() < kFreeSpinCurrentThreshold;
}

void ScoringSubsystem::SpinAmp(double upperPercentage, double lowerPercentage) {
  m_ampLowerSpinnyBoi.Set(lowerPercentage);
  m_ampUpperSpinnyBoi.Set(upperPercentage);
  frc::SmartDashboard::PutNumber("Amp lower target %", lowerPercentage);
}

void ScoringSubsystem::SpinSpeaker(double upperPercentage,
                                   double lowerPercentage) {
  // m_speakerLowerSpinnyBoi.Set(-lowerPercentage);
  // m_speakerUpperSpinnyBoi.Set(-upperPercentage);
  speakerUpperController.RunWithVelocity(-lowerPercentage);
  speakerLowerController.RunWithVelocity(-upperPercentage);
}

void ScoringSubsystem::SpinSubwoofer() {
  m_ampLowerSpinnyBoi.Set(kSubwooferLowerSpeed);
  m_ampUpperSpinnyBoi.Set(kSubwooferUpperSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  ConsoleWriter.logVerbose("Amp Ramp Speed", m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) >= abs(MaxSpeedToRpm(kAmpLowerSpeed));
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  ConsoleWriter.logVerbose("Scoring Subsystem", "Speaker Velocity %.3f",
                           m_speakerLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Speaker Ramp Speed",
                                               m_speakerLowerEnc.GetVelocity());
  return abs(m_speakerLowerEnc.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSpeakerLowerSpeed));
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  ConsoleWriter.logVerbose("Scoring Subsystem", "Subwoofer Velocity %.3f",
                           m_ampLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Subwoofer Ramp Speed",
                                               m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}