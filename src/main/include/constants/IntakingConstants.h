#pragma once

#include <frc/filter/Debouncer.h>
#include <units/angular_velocity.h>

namespace IntakingConstants {
// Change these to match actual values
constexpr double kIntakeSpeed = 0.8;
// Make er' hover!
constexpr double kIntakeAutoSpeed = 0.58;
constexpr double kFeedAmpSpeed = 0.5;
constexpr double kFeedSpeakerSpeed = 1;
constexpr double kFeedSubwooferSpeed = 1;

constexpr double kOutakeSpeed = -0.5;
constexpr double kSecondaryIntakeOutSpeed = -0.05;
constexpr double kPseudoBrakeModeSpeed = 0.1;

constexpr uint8_t kCenterLowerBeamBreakDigitalPort = 3;
constexpr uint8_t kCenterUpperBeamBreakDigitalPort = 1;
constexpr uint8_t kLowerPodiumBeamBreakDigitalPort = 2;
constexpr uint8_t kLowerampBeamBreakDigitalPort = 4;
constexpr uint8_t kUpperPodiumBeamBreakDigitalPort = 6;
constexpr uint8_t kUpperAmpBeamBreakDigitalPort = 5;

constexpr units::second_t kDebounceTime = 50_ms;
constexpr frc::Debouncer::DebounceType kDebounceType =
    frc::Debouncer::DebounceType::kBoth;

constexpr units::revolutions_per_minute_t kMaxRpm = 5676_rpm;

constexpr double kDowntakeSpeed = 0.4;

namespace IntakingPID {
constexpr double kIntakingP = 6e-5;
constexpr double kIntakingI = 1e-6;
constexpr double kIntakingD = 0;
constexpr double kIntakingIZone = 0;
constexpr double kIntakingFF = 0.000015;
}  // namespace IntakingPID
}  // namespace IntakingConstants