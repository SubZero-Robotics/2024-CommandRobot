#include <subsystems/IntakeSubsystem.h>

#include "ColorConstants.h"
#include "moduledrivers/ConnectorX.h"

IntakeSubsystem::IntakeSubsystem() {
  m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, false);
}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
  ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Outaking %s", "");
  m_leftIntakeSpinnyBoy.Set(Intakeconstants::kOutakeSpeed);
}

void IntakeSubsystem::In() {
  //ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Intaking %s", "");
  m_leftIntakeSpinnyBoy.Set(Intakeconstants::kIntakeSpeed);
}

void IntakeSubsystem::Stop() {
  //ConsoleLogger::getInstance().logVerbose("Intake Subsystem", "Stopping %s", "");
  m_leftIntakeSpinnyBoy.Set(0);
}

bool IntakeSubsystem::NotePresent() { return !m_beamBreak.Get(); }
