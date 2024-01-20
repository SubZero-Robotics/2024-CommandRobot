#include <subsystems/IntakeSubsystem.h>
#include "moduledrivers/ConnectorX.h"
#include "ColorConstants.h"

IntakeSubsystem::IntakeSubsystem() {}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
    m_intakeSpinnyBoy.Set(Intakeconstants::kOutakeSpeed);
}

void IntakeSubsystem::In() {
    m_intakeSpinnyBoy.Set(Intakeconstants::kIntakeSpeed);
}

void IntakeSubsystem::Stop() {
    m_intakeSpinnyBoy.Set(0);
}

bool IntakeSubsystem::NotePresent() {
    return m_beamBreak.Get();
}
