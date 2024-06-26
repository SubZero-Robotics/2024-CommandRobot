#include <subsystems/IntakeSubsystem.h>

void IntakeSubsystem::Periodic() {
  ShuffleboardLogger::getInstance().logVerbose("Has Note", NotePresentCenter());

  frc::SmartDashboard::PutBoolean("Note present Lower center ",
                                  NotePresentLowerCenter());
  frc::SmartDashboard::PutBoolean("Note present upper center ",
                                  NotePresentUpperCenter());
  frc::SmartDashboard::PutBoolean("Note present lower amp ",
                                  NotePresentLowerAmp());
  frc::SmartDashboard::PutBoolean("Note present upper amp ",
                                  NotePresentUpperAmp());
  frc::SmartDashboard::PutBoolean("Note present lower podium ",
                                  NotePresentLowerPodium());
  frc::SmartDashboard::PutBoolean("Note present upper podium ",
                                  NotePresentUpperPodium());
}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out(double outakeSpeed) {
  m_leftIntakeSpinnyBoy.Set(outakeSpeed);
}

void IntakeSubsystem::In(double intakeSpeed) {
  m_leftIntakeSpinnyBoy.Set(intakeSpeed);
}

void IntakeSubsystem::Stop() {
  ConsoleWriter.logVerbose("Intake Subsystem", "Stopping %s", "");
  m_leftIntakeSpinnyBoy.Set(0);
}

void IntakeSubsystem::Feed(ScoringDirection direction) {
  switch (direction) {
    case ScoringDirection::AmpSide: {
      In(IntakingConstants::kFeedAmpSpeed);
      break;
    }
    case ScoringDirection::PodiumSide: {
      In(IntakingConstants::kFeedSpeakerSpeed);
      break;
    }
    case ScoringDirection::Subwoofer: {
      In(IntakingConstants::kFeedSubwooferSpeed);
      break;
    }
    case ScoringDirection::FeedPodium: {
      In(IntakingConstants::kFeedSpeakerSpeed);
      break;
    }
  }
}

bool IntakeSubsystem::NotePresentLowerPodium() {
  return !m_lowerPodiumDb.Calculate(m_lowerPodiumBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentUpperAmp() {
  return !m_upperAmpDb.Calculate(m_upperAmpBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentLowerAmp() {
  return !m_lowerAmpDb.Calculate(m_lowerAmpBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentUpperPodium() {
  return !m_upperPodiumDb.Calculate(m_upperPodiumBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentUpperCenter() {
  return !m_centerUpperDb.Calculate(m_centerUpperBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentLowerCenter() {
  return !m_centerLowerDb.Calculate(m_centerLowerBeamBreak.Get());
}

bool IntakeSubsystem::NotePresentCenter() {
  return NotePresentLowerCenter() || NotePresentUpperCenter();
}

bool IntakeSubsystem::NotePresentUpper() {
  return NotePresentUpperAmp() || NotePresentUpperPodium();
}

bool IntakeSubsystem::NotePresentUpperAll() {
  return NotePresentUpper() || NotePresentUpperCenter();
}

bool IntakeSubsystem::NotePresentLower() {
  return NotePresentLowerAmp() || NotePresentLowerPodium();
}

bool IntakeSubsystem::NotePresent() {
  if (m_simDevice) {
    return m_notePresent.Get();
  }

  return NotePresentUpperAmp() || NotePresentLowerPodium() ||
         NotePresentLowerAmp() || NotePresentUpperPodium() ||
         NotePresentCenter();
}