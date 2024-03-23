#include <subsystems/IntakeSubsystem.h>

IntakeSubsystem::IntakeSubsystem() {
  // m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, false);
}

void IntakeSubsystem::Periodic() {
  // intakeTuner.UpdateFromShuffleboard();
  ShuffleboardLogger::getInstance().logVerbose("Has Note", NotePresentCenter());
}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out(double outakeSpeed) {
  // ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Outaking %s",
  //                                         "");

  // intakeMotors.RunWithVelocity(outakeSpeed, outakeSpeed);
  m_leftIntakeSpinnyBoy.Set(outakeSpeed);
}

void IntakeSubsystem::In(double intakeSpeed) {
  // ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Intaking %s",
  //                                         "");
  // intakeMotors.RunWithVelocity(intakeSpeed, intakeSpeed);
  m_leftIntakeSpinnyBoy.Set(intakeSpeed);
}

void IntakeSubsystem::Stop() {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Stopping %s",
                                          "");
  // intakeMotors.Stop();

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
  }
}

bool IntakeSubsystem::NotePresentLowerPodium() {
  return !m_lowerPodiumBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentUpperAmp() {
  return !m_upperAmpBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentLowerAmp() {
  return !m_lowerAmpBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentUpperPodium() {
  return !m_upperPodiumBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentUpperCenter() {
  return !m_centerUpperBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentLowerCenter() {
  return !m_centerLowerBeamBreak.Get();
}

bool IntakeSubsystem::NotePresentCenter() {
  return NotePresentLowerCenter() || NotePresentUpperCenter();
}

bool IntakeSubsystem::NotePresentUpper() {
  return NotePresentUpperAmp() || NotePresentUpperPodium();
}

bool IntakeSubsystem::NotePresentLower() {
  return NotePresentLowerAmp() || NotePresentLowerPodium();
}

bool IntakeSubsystem::NotePresent() {
  return NotePresentUpperAmp() || NotePresentLowerPodium() ||
         NotePresentLowerAmp() || NotePresentUpperPodium() ||
         NotePresentCenter();
}