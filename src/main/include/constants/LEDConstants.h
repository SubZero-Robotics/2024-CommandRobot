#pragma once

#include <units/time.h>

// LED Constants
namespace LEDConstants {
constexpr uint8_t kLedAddress = 23;
constexpr units::second_t kConnectorXDelay = 0.002_s;
constexpr double kAccelThreshold = 1;
}