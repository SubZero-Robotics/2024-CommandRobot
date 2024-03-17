#include "subsystems/ScoringSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {
  // m_speakerLowerSpinnyBoi.Follow(m_speakerUpperSpinnyBoi, false);

  m_ampUpperVelocity = ScoringConstants::kAmpUpperSpeed;
  m_ampLowerVelocity = ScoringConstants::kAmpLowerSpeed;

  frc::SmartDashboard::PutNumber("Amp Upper Velocity", m_ampUpperVelocity);
  frc::SmartDashboard::PutNumber("Amp Lower Velocity", m_ampLowerVelocity);
}

void ScoringSubsystem::Periodic() {
  // speakerTuner.UpdateFromShuffleboard();
  // ampTuner.UpdateFromShuffleboard();

  // speakerUpperTuner.UpdateFromShuffleboard();

  // double latestAmpUpperVelocity =
  //     frc::SmartDashboard::GetNumber("Amp Upper Velocity",
  //     m_ampUpperVelocity);
  // double latestAmpLowerVelocity =
  //     frc::SmartDashboard::GetNumber("Amp Lower Velocity",
  //     m_ampLowerVelocity);

  // if (latestAmpUpperVelocity != m_ampUpperVelocity) {
  //   m_ampUpperVelocity = latestAmpUpperVelocity;
  //   ConsoleLogger::getInstance().logVerbose(
  //       "ScoringSubsystem", "Setting amp UPPER to %f", m_ampUpperVelocity);
  // } else if (latestAmpLowerVelocity != m_ampLowerVelocity) {
  //   m_ampLowerVelocity = latestAmpLowerVelocity;
  //   ConsoleLogger::getInstance().logVerbose(
  //       "ScoringSubsystem", "Setting amp UPPER to %f", m_ampLowerVelocity);
  // }
}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::SpinOutake() {
  // speakerPidPair.RunWithVelocity(ScoringConstants::kScoringOutakeUpperSpeed,
  //                                ScoringConstants::kScoringOutakeLowerSpeed);
  // ampPidPair.RunWithVelocity(ScoringConstants::kScoringOutakeUpperSpeed,
  //                            ScoringConstants::kScoringOutakeLowerSpeed);
  ampUpperController.RunWithVelocity(
      ScoringConstants::kScoringOutakeUpperSpeed);
  ampLowerController.RunWithVelocity(
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
  // ampPidPair.RunWithVelocity(upperPercentage, lowerPercentage);
  ampUpperController.RunWithVelocity(upperPercentage);
  ampLowerController.RunWithVelocity(lowerPercentage);
}

void ScoringSubsystem::SpinSpeaker() {
  // speakerPidPair.RunWithVelocity(kSpeakerUpperSpeed, kSpeakerLowerSpeed);
  speakerUpperController.RunWithVelocity(-kSpeakerUpperSpeed);
  speakerLowerController.RunWithVelocity(-kSpeakerLowerSpeed);
}

void ScoringSubsystem::SpinSubwoofer() {
  // ampPidPair.RunWithVelocity(kSubwooferUpperSpeed, kSubwooferLowerSpeed);
  ampUpperController.RunWithVelocity(kSubwooferUpperSpeed);
  ampLowerController.RunWithVelocity(kSubwooferLowerSpeed);
}

bool ScoringSubsystem::CheckAmpSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose(
      "Scoring Subsystem", "Amp Velocity %.3f", m_ampLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Amp Ramp Speed",
                                               m_ampLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Amp Ramp Speed",
                                               m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) - 750 >=
         abs(MaxSpeedToRpm(kAmpLowerSpeed));
}

bool ScoringSubsystem::CheckSpeakerSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Speaker Velocity %.3f",
                                          m_speakerLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Speaker Ramp Speed",
                                               m_speakerLowerEnc.GetVelocity());
  return abs(m_speakerLowerEnc.GetVelocity()) + 1000 >=
         abs(MaxSpeedToRpm(kSpeakerLowerSpeed));
}

bool ScoringSubsystem::CheckSubwooferSpeed() {
  // TODO: Check range rather than exact equals
  ConsoleLogger::getInstance().logVerbose("Scoring Subsystem",
                                          "Subwoofer Velocity %.3f",
                                          m_ampLowerEnc.GetVelocity());
  ShuffleboardLogger::getInstance().logVerbose("Subwoofer Ramp Speed",
                                               m_ampLowerEnc.GetVelocity());
  return abs(m_ampLowerEnc.GetVelocity()) >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}