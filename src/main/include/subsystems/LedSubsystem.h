#pragma once

#include "moduledrivers/ConnectorX.h"
#include "Constants.h"

using namespace ConnectorX;

class LedSubsystem {
    public:
        LedSubsystem() : m_connectorX(ConnectorXBoard(kLedAddress)) {}

        void intakingLED();
        void scoringSpeakerLED();
        void scoringAmpLED();
        void stowingLED();
        void idlingLED();
        
    private:
        ConnectorXBoard m_connectorX;
};