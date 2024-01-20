#pragma once

#include <frc/util/Color8Bit.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "moduledrivers/ConnectorX.h"
#include "utils/State.h"
#include "Constants.h"

using namespace ConnectorX;

class LedSubsystem : public frc2::SubsystemBase {
    public:
        LedSubsystem() : m_connectorX(ConnectorXBoard(kLedAddress)) {
            m_connectorX.setLedPort(ConnectorX::LedPort::P1);
        }

        void Periodic() override;

        void SimulationPeriodic() override;

        frc2::CommandPtr ShowFromState(StateGetter stateGetter);

        frc2::CommandPtr Intaking();
        frc2::CommandPtr ScoringSpeaker();
        frc2::CommandPtr ScoringAmp();
        frc2::CommandPtr Stowing();
        frc2::CommandPtr Idling();

    private:
        frc2::CommandPtr setColorAndPattern(LedPort port, frc::Color8Bit color,
            PatternType pattern, bool oneShot = false, int16_t delay = -1);
        
        ConnectorXBoard m_connectorX;
};