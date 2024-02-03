#pragma once

#include <frc/util/Color.h>

#include "moduledrivers/ConnectorX.h"

namespace ColorConstants {
static const frc::Color8Bit kYellow{255, 255, 0};
static const frc::Color8Bit kPurple{110, 10, 250};
static const frc::Color8Bit kRed(0xFF, 0x00, 0x00);
static const frc::Color8Bit kGreen(0x00, 0xFF, 0x00);
static const frc::Color8Bit kBlue(0x00, 0x00, 0xFF);
}  // namespace ColorConstants

namespace LedConstants {
constexpr ConnectorX::LedPort kIntakeLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kSpeakerLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kAmpLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kStowLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kIdleLedPort = ConnectorX::LedPort::P1;
}  // namespace LedConstants