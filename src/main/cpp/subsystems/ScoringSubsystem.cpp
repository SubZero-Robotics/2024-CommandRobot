#include "subsystems/ScoringSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {
  // m_speakerLowerSpinnyBoi.Follow(m_speakerUpperSpinnyBoi, false);
}

void ScoringSubsystem::Periodic() {
  speakerTuner.UpdateFromShuffleboard();
  ampTuner.UpdateFromShuffleboard();
}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::SpinOutake() {
  speakerSideMotors.RunWithVelocity(ScoringConstants::kScoringOutakeUpperSpeed,
                                    ScoringConstants::kScoringOutakeLowerSpeed);
  ampSideMotors.RunWithVelocity(ScoringConstants::kScoringOutakeUpperSpeed,
                                ScoringConstants::kScoringOutakeLowerSpeed);
};

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

void ScoringSubsystem::SpinAmp(double upperPercentage, double lowerPercentage) {
  ampSideMotors.RunWithVelocity(upperPercentage, lowerPercentage);
}

void ScoringSubsystem::SpinSpeaker() {
  speakerSideMotors.RunWithVelocity(kSpeakerUpperSpeed, kSpeakerLowerSpeed);
}

void ScoringSubsystem::SpinSubwoofer() {
  ampSideMotors.RunWithVelocity(kSubwooferUpperSpeed, kSubwooferLowerSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose(
      "Scoring Subsystem", "Amp Velocity %.3f", m_ampEncoder.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Amp Ramp Speed",
                                               m_ampEncoder.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Amp Ramp Speed",
                                               m_ampEncoder.GetVelocity());
  return abs(m_ampEncoder.GetVelocity()) - 750 >=
         abs(MaxSpeedToRpm(kAmpLowerSpeed));
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Speaker Velocity %.3f",
                                          m_speakerEncoder.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Speaker Ramp Speed",
                                               m_speakerEncoder.GetVelocity());
  return abs(m_speakerEncoder.GetVelocity()) + 1000 >=
         abs(MaxSpeedToRpm(kSpeakerLowerSpeed));
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Subwoofer Velocity %.3f",
                                          m_ampEncoder.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Subwoofer Ramp Speed",
                                               m_ampEncoder.GetVelocity());
  return abs(m_ampEncoder.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}