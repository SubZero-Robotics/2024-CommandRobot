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
    // Is this better than Set(0)?
    m_intakeSpinnyBoy.StopMotor();
}

bool IntakeSubsystem::NotePresent() {
    return m_beamBreak.Get();
}
