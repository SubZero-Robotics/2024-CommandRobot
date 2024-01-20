#include <subsystems/IntakeSubsystem.h>
#include "moduledrivers/ConnectorX.h"
#include "ColorConstants.h"

IntakeSubsystem::IntakeSubsystem() {}

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::Out() {
    // if (m_ledSubsystem->getCurrentColor(ConnectorX::LedPort::P1) == ColorConstants::kYellow) {
    //     m_intakeSpinnyBoy.Set(ArmConstants::kOuttakeSpeed);
    // } else {
    //     m_intakeSpinnyBoy.Set(-ArmConstants::kOuttakeSpeed);
    // }
}

void IntakeSubsystem::In(IntakeDirection direction) {
    // if (m_ledSubsystem->getCurrentColor(ConnectorX::LedPort::P1) == ColorConstants::kPurple) {
    //     m_intakeSpinnyBoy.Set(-ArmConstants::kIntakeSpeed);
    // } else {
    //     m_intakeSpinnyBoy.Set(ArmConstants::kIntakeSpeed);
    // }
}

void IntakeSubsystem::Stop() {
    m_intakeSpinnyBoy.Set(0.0);
    m_vectorSpinnyboy.Set(0.0);
}