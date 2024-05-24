#pragma once

#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "ColorConstants.h"

namespace ArmConstants {
// Motor Constants
constexpr int kTicksPerMotorRotation = 42;
constexpr int kReverseRotationsSoftLimit = 0;

// Homing Speeds
constexpr double kRotationHomingSpeed = .15;

// Arm Constants
constexpr int kArmGearRatio = 125;
constexpr units::degree_t kArmRelativeDistancePerRev = 360_deg * (1 / 8.75);
constexpr units::degree_t kArmAbsoluteDistancePerRev = 360_deg;
constexpr units::degree_t kAmpRotation = 142_deg;
constexpr units::degree_t kMaxRotation = 190_deg;
constexpr units::degree_t kHomeRotation = 10_deg;
constexpr units::degrees_per_second_t kDefaultVelocity = 10_deg_per_s;
constexpr double kVelocityScalar = 1.0;
constexpr units::degree_t kTolerance = 2_deg;
constexpr units::meter_t kArmLength = 0.2_m;

constexpr double kArmP = 0.075;
constexpr double kArmI = 0;
constexpr double kArmD = 0;
constexpr double kArmIZone = 0;
constexpr double kArmFF = 0;

static const subzero::SingleAxisMechanism kArmMechanism = {
    // length
    0.2_m,
    // min angle
    110_deg,
    // line width
    6,
    // color
    ColorConstants::kBlue};
}  // namespace ArmConstants