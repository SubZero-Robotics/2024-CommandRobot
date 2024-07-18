#pragma once

#include <units/length.h>
#include <units/voltage.h>

namespace CharacterizationConstants {
namespace FrontLeftDrive {
constexpr units::volt_t kS = 0.43779_V;
constexpr auto kV = 2.4439_V / 1_mps;
constexpr auto kA = 0.36441_V / 1_mps_sq;
}  // namespace FrontLeftDrive

namespace FrontRightDrive {
constexpr units::volt_t kS = 0.47478_V;
constexpr auto kV = 2.4511_V / 1_mps;
constexpr auto kA = 0.36441_V / 1_mps_sq;
}  // namespace FrontRightDrive

namespace RearLeftDrive {
constexpr units::volt_t kS = 0.43972_V;
constexpr auto kV = 2.4352_V / 1_mps;
constexpr auto kA = 0.35057_V / 1_mps_sq;
}  // namespace RearLeftDrive

namespace RearRightDrive {
constexpr units::volt_t kS = 0.41185_V;
constexpr auto kV = 2.4218_V / 1_mps;
constexpr auto kA = 0.41332_V / 1_mps_sq;
}  // namespace RearRightDrive
}  // namespace CharacterizationConstants