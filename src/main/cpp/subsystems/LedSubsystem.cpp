#include "subsystems/LedSubsystem.h"
#include "ColorConstants.h"

void LedSubsystem::intakingLED() {
    m_connectorX.setColor(LedConstants::kIntakeLedPort, ColorConstants::kRed);
    m_connectorX.setPattern(LedConstants::kIntakeLedPort, PatternType::Blink);
}

void LedSubsystem::scoringSpeakerLED() {
    m_connectorX.setColor(LedConstants::kSpeakerLedPort, ColorConstants::kGreen);
    m_connectorX.setPattern(LedConstants::kSpeakerLedPort, PatternType::Blink);
}

void LedSubsystem::scoringAmpLED() {
    m_connectorX.setColor(LedConstants::kAmpLedPort, ColorConstants::kBlue);
    m_connectorX.setPattern(LedConstants::kAmpLedPort, PatternType::Blink);
}

void LedSubsystem::stowingLED() {
    m_connectorX.setColor(LedConstants::kStowLedPort, ColorConstants::kPurple);
    m_connectorX.setPattern(LedConstants::kStowLedPort, PatternType::Breathe);
}

void LedSubsystem::idlingLED() {
    m_connectorX.setColor(LedConstants::kIdleLedPort, ColorConstants::kYellow);
    m_connectorX.setPattern(LedConstants::kIdleLedPort, PatternType::SetAll);
}