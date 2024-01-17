#include "subsystems/LedSubsystem.h"
#include "ColorConstants.h"

void LedSubsystem::intakingLED() {
    m_connectorX.setColor(LedConstants::kIntakeLedPort, ColorConstants::kRed);
}

void LedSubsystem::scoringSpeakerLED() {
    m_connectorX.setColor(LedConstants::kSpeakerLedPort, ColorConstants::kGreen);
}

void LedSubsystem::scoringAmpLED() {
    m_connectorX.setColor(LedConstants::kAmpLedPort, ColorConstants::kBlue);
}

void LedSubsystem::stowingLED() {
    m_connectorX.setColor(LedConstants::kStowLedPort, ColorConstants::kPurple);
}

void LedSubsystem::idlingLED() {
    m_connectorX.setColor(LedConstants::kIdleLedPort, ColorConstants::kYellow);
}