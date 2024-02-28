#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem() {
  // m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, false);
}

void IntakeSubsystem::Periodic() { intakeTuner.UpdateFromShuffleboard(); }

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out(double outakeSpeed) {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Outaking %s",
                                          "");

  intakeMotors.RunWithVelocity(outakeSpeed, outakeSpeed);
}

void IntakeSubsystem::In(double intakeSpeed) {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Intaking %s",
                                          "");
  intakeMotors.RunWithVelocity(intakeSpeed, intakeSpeed);
}

void IntakeSubsystem::Stop() {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Stopping %s",
                                          "");
  intakeMotors.Stop();
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