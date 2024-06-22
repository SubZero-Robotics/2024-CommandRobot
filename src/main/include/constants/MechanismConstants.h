#pragma once

#include <units/angular_velocity.h>

namespace MechanismConstants {
constexpr double kClimberLeftX = 0.2;
constexpr double kClimberLeftY = 0;
constexpr double kClimberRightX = 0.2;
constexpr double kClimberRightY = 0.2;
constexpr double kArmRootX = 0.2;
constexpr double kArmRootY = 0.1;
constexpr double kArmPostX = 0.2;
constexpr units::degree_t kArmPostAngle = 90_deg;
}  // namespace MechanismConstants