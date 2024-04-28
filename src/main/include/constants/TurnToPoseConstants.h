#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

namespace TurnToPoseConstants {
constexpr units::degrees_per_second_t kProfileVelocity = 960_deg_per_s;
constexpr units::degrees_per_second_squared_t kProfileAcceleration =
    1200_deg_per_s_sq;
constexpr double kTurnP = 5;
constexpr double kTurnI = 0;
constexpr double kTurnD = 0;
constexpr frc::Pose2d kPoseTolerance =
    frc::Pose2d(0.05_m, 0.05_m, frc::Rotation2d(0.5_deg));
constexpr double kBlendRatio = 0.1;

constexpr double kTurnTranslationP = 1;
constexpr double kTurnTranslationI = 0;
constexpr double kTurnTranslationD = 0;
}  // namespace TurnToPoseConstants