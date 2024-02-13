#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem() {
  m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, false);
}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Outaking %s",
                                          "");
  m_leftIntakeSpinnyBoy.Set(Intakeconstants::kOutakeSpeed);
}

void IntakeSubsystem::In(double intakeSpeed) {
  // ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Intaking %s",
  // "");
  m_leftIntakeSpinnyBoy.Set(intakeSpeed);
}

void IntakeSubsystem::Stop() {
  // ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Stopping %s",
  // "");
  m_leftIntakeSpinnyBoy.Set(0);
}

void IntakeSubsystem::Feed(ScoringDirection direction) {
  switch (direction) {
    case ScoringDirection::AmpSide: {
      In(Intakeconstants::kFeedAmpSpeed);
    }
    case ScoringDirection::SpeakerSide: {
      In(Intakeconstants::kFeedSpeakerSpeed);
    }
    case ScoringDirection::Subwoofer: {
      In(Intakeconstants::kFeedSubwooferSpeed);
    }
  }
}

bool IntakeSubsystem::NotePresent() { return !m_beamBreak.Get(); }
