#include "subsystems/ScoringSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ScoringConstants;

ScoringSubsystem::ScoringSubsystem() {
  // m_speakerLowerSpinnyBoi.Follow(m_speakerUpperSpinnyBoi, false);

  tuningMotor = "Speaker Upper";

  m_speakerUpperPidController.SetP(ScoringPID::kSpeakerUpperP);
  m_speakerUpperPidController.SetI(ScoringPID::kSpeakerUpperI);
  m_speakerUpperPidController.SetD(ScoringPID::kSpeakerUpperD);
  m_speakerUpperPidController.SetIZone(ScoringPID::kSpeakerUpperIZone);
  m_speakerUpperPidController.SetFF(ScoringPID::kSpeakerUpperFF);

  m_speakerLowerPidController.SetP(ScoringPID::kSpeakerLowerP);
  m_speakerLowerPidController.SetI(ScoringPID::kSpeakerLowerI);
  m_speakerLowerPidController.SetD(ScoringPID::kSpeakerLowerD);
  m_speakerLowerPidController.SetIZone(ScoringPID::kSpeakerLowerIZone);
  m_speakerLowerPidController.SetFF(ScoringPID::kSpeakerLowerFF);

  m_ampUpperPidController.SetP(ScoringPID::kAmpUpperP);
  m_ampUpperPidController.SetI(ScoringPID::kAmpUpperI);
  m_ampUpperPidController.SetD(ScoringPID::kAmpUpperD);
  m_ampUpperPidController.SetIZone(ScoringPID::kAmpUpperIZone);
  m_ampUpperPidController.SetFF(ScoringPID::kAmpUpperFF);

  m_ampLowerPidController.SetP(ScoringPID::kAmpLowerP);
  m_ampLowerPidController.SetI(ScoringPID::kAmpLowerI);
  m_ampLowerPidController.SetD(ScoringPID::kAmpLowerD);
  m_ampLowerPidController.SetIZone(ScoringPID::kAmpLowerIZone);
  m_ampLowerPidController.SetFF(ScoringPID::kAmpLowerFF);

  scoringPIDs["Speaker Upper"] = {
      .name = "Speaker Upper",
      .P = ScoringPID::kSpeakerUpperP,
      .I = ScoringPID::kSpeakerUpperI,
      .D = ScoringPID::kSpeakerUpperD,
      .IZone = ScoringPID::kSpeakerUpperIZone,
      .FF = ScoringPID::kSpeakerUpperFF,
      .velocity = ScoringPID::kSpeakerUpperVelocity};
  scoringPIDs["Speaker Lower"] = {
      .name = "Speaker Lower",
      .P = ScoringPID::kSpeakerLowerP,
      .I = ScoringPID::kSpeakerLowerI,
      .D = ScoringPID::kSpeakerLowerD,
      .IZone = ScoringPID::kSpeakerLowerIZone,
      .FF = ScoringPID::kSpeakerLowerFF,
      .velocity = ScoringPID::kSpeakerLowerVelocity};
  scoringPIDs["Amp Upper"] = {.name = "Amp Upper",
                              .P = ScoringPID::kAmpUpperP,
                              .I = ScoringPID::kAmpUpperI,
                              .D = ScoringPID::kAmpUpperD,
                              .IZone = ScoringPID::kAmpUpperIZone,
                              .FF = ScoringPID::kAmpUpperFF,
                              .velocity = ScoringPID::kAmpUpperVelocity};
  scoringPIDs["Amp Lower"] = {.name = "Amp Lower",
                              .P = ScoringPID::kAmpLowerP,
                              .I = ScoringPID::kAmpLowerI,
                              .D = ScoringPID::kAmpLowerD,
                              .IZone = ScoringPID::kAmpLowerIZone,
                              .FF = ScoringPID::kAmpLowerFF,
                              .velocity = ScoringPID::kAmpLowerVelocity};

  if (scoringPIDs.find(tuningMotor) == scoringPIDs.end()) {
    ConsoleLogger::getInstance().logError(
        "PID Error:", "%s not in PID settings.", tuningMotor);
  }

  frc::SmartDashboard::PutNumber(tuningMotor + " P Gain",
                                 scoringPIDs[tuningMotor].P);
  frc::SmartDashboard::PutNumber(tuningMotor + " I Gain",
                                 scoringPIDs[tuningMotor].I);
  frc::SmartDashboard::PutNumber(tuningMotor + " D Gain",
                                 scoringPIDs[tuningMotor].D);
  frc::SmartDashboard::PutNumber(tuningMotor + " I Zone",
                                 scoringPIDs[tuningMotor].IZone);
  frc::SmartDashboard::PutNumber(tuningMotor + " Feed Forward",
                                 scoringPIDs[tuningMotor].FF);
  frc::SmartDashboard::PutNumber(tuningMotor + " Velocity",
                                 scoringPIDs[tuningMotor].velocity);

  tuningLatestP = scoringPIDs[tuningMotor].P;
  tuningLatestI = scoringPIDs[tuningMotor].I;
  tuningLatestD = scoringPIDs[tuningMotor].D;
  tuningLatestIZone = scoringPIDs[tuningMotor].IZone;
  tuningLatestFF = scoringPIDs[tuningMotor].FF;
  tuningLatestVelocity = scoringPIDs[tuningMotor].velocity;
}

void ScoringSubsystem::Periodic() {
  double tP = frc::SmartDashboard::GetNumber(tuningMotor + " P Gain", 0);
  double tI = frc::SmartDashboard::GetNumber(tuningMotor + " Tuning I Gain", 0);
  double tD = frc::SmartDashboard::GetNumber(tuningMotor + " Tuning D Gain", 0);
  double tIZ = frc::SmartDashboard::GetNumber(tuningMotor + " I Zone", 0);
  double tFF = frc::SmartDashboard::GetNumber(tuningMotor + " Feed Forward", 0);
  double tV = frc::SmartDashboard::GetNumber(tuningMotor + " Velocity", 0);

  if (tP != tuningLatestP) {
    m_tuningPidControllerUpper->SetP(tP);
    m_tuningPidControllerLower->SetP(tP);
    scoringPIDs[tuningMotor].P = tP;
    tuningLatestP = tP;
  }
  if (tI != tuningLatestI) {
    m_tuningPidControllerUpper->SetI(tI);
    m_tuningPidControllerLower->SetI(tI);
    scoringPIDs[tuningMotor].I = tI;
    tuningLatestI = tI;
  }
  if (tD != tuningLatestD) {
    m_tuningPidControllerUpper->SetD(tD);
    m_tuningPidControllerLower->SetD(tD);
    scoringPIDs[tuningMotor].D = tD;
    tuningLatestD = tD;
  }
  if (tIZ != tuningLatestIZone) {
    m_tuningPidControllerUpper->SetIZone(tIZ);
    m_tuningPidControllerLower->SetIZone(tIZ);
    scoringPIDs[tuningMotor].IZone = tIZ;
    tuningLatestIZone = tIZ;
  }
  if (tFF != tuningLatestFF) {
    m_tuningPidControllerUpper->SetFF(tFF);
    m_tuningPidControllerLower->SetFF(tFF);
    scoringPIDs[tuningMotor].FF = tFF;
    tuningLatestFF = tFF;
  }
  if (tV != tuningLatestVelocity) {
    scoringPIDs[tuningMotor].velocity = tV;
    tuningLatestVelocity = tV;
  }
}

void ScoringSubsystem::SimulationPeriodic() {}

void ScoringSubsystem::SpinOutake() {
  m_ampLowerSpinnyBoi.Set(ScoringConstants::kScoringOutakeLowerSpeed);
  m_ampUpperSpinnyBoi.Set(ScoringConstants::kScoringOutakeUpperSpeed);
  m_speakerUpperSpinnyBoi.Set(ScoringConstants::kScoringOutakeUpperSpeed);
  m_speakerLowerSpinnyBoi.Set(ScoringConstants::kScoringOutakeLowerSpeed);
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
  m_ampLowerPidController.SetReference(
      MaxSpeedToRpm(scoringPIDs["Amp Lower"].velocity),
      rev::CANSparkMax::ControlType::kVelocity);
  m_ampUpperPidController.SetReference(
      MaxSpeedToRpm(scoringPIDs["Amp Upper"].velocity),
      rev::CANSparkMax::ControlType::kVelocity);
  // m_ampLowerSpinnyBoi.Set(kAmpLowerSpeed);
  // m_ampUpperSpinnyBoi.Set(kAmpUpperSpeed);
}

void ScoringSubsystem::SpinSpeaker() {
  m_speakerUpperPidController.SetReference(
      MaxSpeedToRpm(scoringPIDs["Speaker Upper"].velocity),
      rev::CANSparkMax::ControlType::kVelocity);
  m_speakerLowerPidController.SetReference(
      MaxSpeedToRpm(scoringPIDs["Speaker Lower"].velocity),
      rev::CANSparkMax::ControlType::kVelocity);
  // m_speakerUpperSpinnyBoi.Set(kSpeakerUpperSpeed);
  // m_speakerLowerSpinnyBoi.Set(kSpeakerLowerSpeed);
}

void ScoringSubsystem::SpinSubwoofer() {
  m_ampLowerPidController.SetReference(
      MaxSpeedToRpm(kSubwooferLowerSpeed),
      rev::CANSparkMax::ControlType::kVelocity);
  m_ampUpperPidController.SetReference(
      MaxSpeedToRpm(kSubwooferUpperSpeed),
      rev::CANSparkMax::ControlType::kVelocity);
  // m_ampLowerSpinnyBoi.Set(kSubwooferLowerSpeed);
  // m_ampUpperSpinnyBoi.Set(kSubwooferUpperSpeed);
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
  return abs(m_ampEncoder.GetVelocity()) + 1500 >=
         abs(MaxSpeedToRpm(kSubwooferLowerSpeed));
}