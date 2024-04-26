#pragma once

#include <units/angular_velocity.h>

namespace MechanismConstants {
constexpr double kClimberLeftX = 0.5;
constexpr double kClimberLeftY = 0;
constexpr double kClimberRightX = 0.5;
constexpr double kClimberRightY = 1;
constexpr double kArmRootX = 0.5;
constexpr double kArmRootY = 0.5;
constexpr double kArmPostX = 0.4;
constexpr units::degree_t kArmPostAngle = 90_deg;
}  // namespace MechanismConstants