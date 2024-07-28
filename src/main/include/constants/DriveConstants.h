#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 3.9_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
#ifdef TEST_SWERVE_BOT
constexpr units::meter_t kTrackWidth =
    0.5588_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    0.5588_m;  // Distance between centers of front and back wheels on robot
#else
constexpr units::meter_t kTrackWidth =
    24_in;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    24_in;  // Distance between centers of front and back wheels on robot
#endif
// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset = -std::numbers::pi / 2;
constexpr double kFrontRightChassisAngularOffset = 0;
constexpr double kRearLeftChassisAngularOffset = std::numbers::pi;
constexpr double kRearRightChassisAngularOffset = std::numbers::pi / 2;

// SPARK MAX CAN IDs
constexpr int kFrontRightDrivingCanId = 2;
constexpr int kRearRightDrivingCanId = 4;
constexpr int kFrontLeftDrivingCanId = 8;
constexpr int kRearLeftDrivingCanId = 6;

constexpr int kFrontRightTurningCanId = 1;
constexpr int kRearRightTurningCanId = 3;
constexpr int kFrontLeftTurningCanId = 7;
constexpr int kRearLeftTurningCanId = 5;

constexpr units::second_t kLoopTime = 0.022_s;
constexpr double kLoopsPerSecond = 1_s / kLoopTime;

const std::vector<std::string> kMotorNames = {
    "Front-Left-Drive", "Rear-Left-Drive", "Front-Right-Drive",
    "Rear-Right-Drive"};

constexpr double kSmallAngleDif = 0.45 * std::numbers::pi;
constexpr double kLargeAngleDif = 0.85 * std::numbers::pi;
constexpr double kPlaceHolderSlewRate = 500.0;
}  // namespace DriveConstants