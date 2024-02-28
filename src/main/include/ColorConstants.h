#pragma once

#include <frc/util/Color.h>

#include "moduledrivers/ConnectorX.h"

namespace ColorConstants {
static const frc::Color8Bit kYellow{0xE5, 0xD5, 0x24};
static const frc::Color8Bit kPurple{0x57, 0x14, 0xE8};
static const frc::Color8Bit kRed{0xFC, 0x19, 0x19};
static const frc::Color8Bit kGreen{0x28, 0xCC, 0x2E};
static const frc::Color8Bit kBlue{0x08, 0x3B, 0xD3};
static const frc::Color8Bit kTeal{0x18, 0x9B, 0xCE};
static const frc::Color8Bit kOrange{0xE8, 0x61, 0x19};
}  // namespace ColorConstants

namespace LedConstants {
constexpr ConnectorX::LedPort kIntakeLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kSpeakerLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kAmpLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kStowLedPort = ConnectorX::LedPort::P1;
constexpr ConnectorX::LedPort kIdleLedPort = ConnectorX::LedPort::P1;
}  // namespace LedConstants