#include <subsystems/IntakeSubsystem.h>
#include "moduledrivers/ConnectorX.h"
#include "ColorConstants.h"

IntakeSubsystem::IntakeSubsystem() {
    m_rightIntakeSpinnyBoy.Follow(m_leftIntakeSpinnyBoy, true);
}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
    m_leftIntakeSpinnyBoy.Set(Intakeconstants::kOutakeSpeed);
}

void IntakeSubsystem::In() {
    m_leftIntakeSpinnyBoy.Set(Intakeconstants::kIntakeSpeed);
}

void IntakeSubsystem::Stop() {
    m_leftIntakeSpinnyBoy.Set(0);
}

bool IntakeSubsystem::NotePresent() {
    return m_beamBreak.Get();
}
