#include <subsystems/IntakeSubsystem.h>
#include "moduledrivers/ConnectorX.h"

void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::SimulationPeriodic() {}

void IntakeSubsystem::In(RobotState) {
    m_intakeSpinnyBoy.Set(Intakeconstants::kIntakeSpeed);
    m_vectorSpinnyboy.Set(Intakeconstants::kIntakeVectorspeed);
}

void IntakeSubsystem::Out(RobotState) {
    m_intakeSpinnyBoy.Set(-Intakeconstants::kOutakeSpeed);
    m_vectorSpinnyboy.Set(-Intakeconstants::kOutakeVectorspeed);
}

void IntakeSubsystem::Stop() {
    m_intakeSpinnyBoy.Set(0.0);
    m_vectorSpinnyboy.Set(0.0);
}