#pragma once

#include <subzero/singleaxis/ISingleAxisSubsystem.h>
#include <units/length.h>
#include <units/velocity.h>

#include "ColorConstants.h"

namespace ClimbConstants {
constexpr int kClimberLeftMotorId = 10;
constexpr int kClimberRightMotorId = 11;

constexpr double kClimberSetP = 40.0;
constexpr double kClimberSetI = 0.0;
constexpr double kClimberSetD = 0.0;
constexpr double kClimberSetIZone = 0.0;
constexpr double kClimberSetFF = 0.0;

// Minimum climber extension distance
constexpr units::inch_t kMinClimberDistance = 0_in;
// Maximum climber extension distance
constexpr units::inch_t kMaxClimberDistance = 36_in;
// Extended position
constexpr units::inch_t kClimbExtensionPosition = 20_in;
// Retracted position
constexpr units::inch_t kClimbRetractPosition = 1_in;
constexpr units::inch_t kInPerRotation = 1_in / 23.1;
constexpr units::inch_t kClimberTolerance = 0.5_in;

constexpr double kClimbStepSize = 1.0;
constexpr double kClimbHomingSpeed = 1.0;
constexpr int kTicksPerMotorRotation = 42;

constexpr units::feet_per_second_t kClimberExtendSpeed = 1_fps;
constexpr double kClimberVelocityScalar = 1.0;

// Distance between left and right arm centers
constexpr units::meter_t kClimberOffsetDistance = 4_m;

static const subzero::SingleAxisMechanism kLeftClimberMechanism = {
    // min length
    24_in,
    // angle
    70_deg,
    // line width
    6.0,
    // color
    ColorConstants::kGreen};

static const subzero::SingleAxisMechanism kRightClimberMechanism = {
    // min length
    24_in,
    // angle
    70_deg,
    // line width
    6.0,
    // color
    ColorConstants::kRed};
}  // namespace ClimbConstants