#include <subsystems/IntakeSubsystem.h>

#include "ColorConstants.h"
#include "moduledrivers/ConnectorX.h"

IntakeSubsystem::IntakeSubsystem() {
    m_rearIntakeSpinnyBoy.Follow(m_frontIntakeSpinnyBoy, true);
}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
    m_frontIntakeSpinnyBoy.Set(Intakeconstants::kOutakeSpeed);
}

void IntakeSubsystem::In() {
    m_frontIntakeSpinnyBoy.Set(Intakeconstants::kIntakeSpeed);
}

void IntakeSubsystem::Stop() {
    m_frontIntakeSpinnyBoy.Set(0);
}

bool IntakeSubsystem::NotePresent() {
    return !m_beamBreak.Get();
}
