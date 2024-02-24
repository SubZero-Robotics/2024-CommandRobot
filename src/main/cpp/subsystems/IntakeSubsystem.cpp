#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem() {
  // m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, false);
}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out(double outakeSpeed) {
  // ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Outaking %s",
  //                                         "");
  m_leftIntakeSpinnyBoy.Set(outakeSpeed);
  m_rightIntakeSpinnyBoy.Set(outakeSpeed);
}

void IntakeSubsystem::In(double intakeSpeed) {
 //ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Intaking %s",
  //                                        "");
  m_leftIntakeSpinnyBoy.Set(intakeSpeed);
  m_rightIntakeSpinnyBoy.Set(intakeSpeed);
}

void IntakeSubsystem::Stop() {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Stopping %s",
                                          "");
  m_leftIntakeSpinnyBoy.Set(0);
  m_rightIntakeSpinnyBoy.Set(0);
}

void IntakeSubsystem::Feed(ScoringDirection direction) {
  switch (direction) {
    case ScoringDirection::AmpSide: {
      In(IntakingConstants::kFeedAmpSpeed);
      break;
    }
    case ScoringDirection::SpeakerSide: {
      In(IntakingConstants::kFeedSpeakerSpeed);
      break;
    }
    case ScoringDirection::Subwoofer: {
      In(IntakingConstants::kFeedSubwooferSpeed);
      break;
    }
  }
}

bool IntakeSubsystem::NotePresentLower() { return !m_lowerBeamBreak.Get(); }

bool IntakeSubsystem::NotePresentUpper() { return !m_upperBeamBreak.Get(); }

bool IntakeSubsystem::NotePresent() {
  return NotePresentUpper() || NotePresentLower();
}