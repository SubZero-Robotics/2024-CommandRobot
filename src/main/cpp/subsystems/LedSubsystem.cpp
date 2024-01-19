#include "subsystems/LedSubsystem.h"
#include "ColorConstants.h"

void LedSubsystem::intakingLedColor() {
    m_connectorX.setColor(LedConstants::kIntakeLedPort, ColorConstants::kRed);
}

void LedSubsystem::scoringSpeakerLedColor() {
    m_connectorX.setColor(LedConstants::kSpeakerLedPort, ColorConstants::kGreen);
}

void LedSubsystem::scoringAmpLedColor() {
    m_connectorX.setColor(LedConstants::kAmpLedPort, ColorConstants::kBlue);
}

void LedSubsystem::stowingLedColor() {
    m_connectorX.setColor(LedConstants::kStowLedPort, ColorConstants::kPurple);
}

void LedSubsystem::idlingLedColor() {
    m_connectorX.setColor(LedConstants::kIdleLedPort, ColorConstants::kYellow);
}

void LedSubsystem::intakingLedPattern() {
    m_connectorX.setPattern(LedConstants::kIntakeLedPort, PatternType::Blink);
}

void LedSubsystem::scoringSpeakerLedPattern() {
    m_connectorX.setPattern(LedConstants::kSpeakerLedPort, PatternType::Blink);
}

void LedSubsystem::scoringAmpLedPattern() {
    m_connectorX.setPattern(LedConstants::kAmpLedPort, PatternType::Blink);
}

void LedSubsystem::stowingLedPattern() {
    m_connectorX.setPattern(LedConstants::kStowLedPort, PatternType::Breathe);
}

void LedSubsystem::idlingLedPattern() {
    m_connectorX.setPattern(LedConstants::kIdleLedPort, PatternType::SetAll);
}