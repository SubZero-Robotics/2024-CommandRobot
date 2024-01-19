#pragma once

#include "moduledrivers/ConnectorX.h"
#include "Constants.h"

using namespace ConnectorX;

class LedSubsystem {
    public:
        LedSubsystem() : m_connectorX(ConnectorXBoard(kLedAddress)) {
            m_connectorX.setLedPort(ConnectorX::LedPort::P1);
        }

        void intakingLedColor();
        void scoringSpeakerLedColor();
        void scoringAmpLedColor();
        void stowingLedColor();
        void idlingLedColor();

        void intakingLedPattern();
        void scoringSpeakerLedPattern();
        void scoringAmpLedPattern();
        void stowingLedPattern();
        void idlingLedPattern();
        
        ConnectorXBoard m_connectorX;
};