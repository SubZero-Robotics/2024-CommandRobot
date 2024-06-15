#pragma once

#include <subzero/moduledrivers/ConnectorX.h>
#include <units/time.h>

// LED Constants
namespace LEDConstants {
constexpr uint8_t kLedAddress = 23;
constexpr units::second_t kConnectorXDelay = 0.002_s;
constexpr double kAccelThreshold = 1;
constexpr ConnectorX::LedPort kIntakeLedPort = ConnectorX::LedPort::P0;
constexpr ConnectorX::LedPort kIdleLedPort = ConnectorX::LedPort::P0;
}  // namespace LEDConstants