#pragma once

#include <units/angular_velocity.h>

#include <string>

enum class ScoringDirection { AmpSide = 0, PodiumSide, Subwoofer, FeedPodium };

namespace ScoringConstants {
constexpr double kFreeSpinCurrentThreshold = 90;
constexpr units::revolutions_per_minute_t kMaxSpinRpm = 6784_rpm;

constexpr double kShuffleSpeed = 0.05;

// Positive = clockwise
constexpr double kVectorSpeed = -0.4;
constexpr double kSubwooferVectorSpeed = 1;
constexpr double kFeedPodiumVectorSpeed = -1;

// These need to be different
constexpr double kAmpLowerSpeed = -0.254 * 1.7;
constexpr double kAmpUpperSpeed = -0.168 * 1.3;

// These should match
constexpr double kSpeakerLowerSpeed = 1;
constexpr double kSpeakerUpperSpeed = kSpeakerLowerSpeed;

// These should also match
constexpr double kSubwooferLowerSpeed = -0.95;
constexpr double kSubwooferUpperSpeed = kSubwooferLowerSpeed;

// nice :flushed:
constexpr double kFeedLowerSpeed = 0.69;
constexpr double kFeedUpperSpeed = 0.69;

constexpr double kScoringOutakeUpperSpeed = 0.2;
constexpr double kScoringOutakeLowerSpeed = kScoringOutakeUpperSpeed;

constexpr double kScoringIntakingOutakeUpperSpeed = 0.2;
constexpr double kScoringIntakingOutakeLowerSpeed =
    kScoringIntakingOutakeUpperSpeed;

enum class ScoreState {
  FlywheelRamp,
  Feeding,
  Shooting,
};

constexpr units::second_t kFlywheelRampDelay = 0.5_s;

namespace ScoringPID {
constexpr double kSpeakerP = 4e-5;
constexpr double kSpeakerI = 1e-6;
constexpr double kSpeakerD = 0;
constexpr double kSpeakerIZone = 0;
constexpr double kSpeakerFF = 0.000015;

// constexpr double kSpeakerLowerP = 6e-5;
// constexpr double kSpeakerLowerI = 1e-6;
// constexpr double kSpeakerLowerD = 0;
// constexpr double kSpeakerLowerIZone = 0;
// constexpr double kSpeakerLowerFF = 0.000015;
// constexpr double kSpeakerLowerVelocity = -1;

constexpr double kAmpP = 6e-5;
constexpr double kAmpI = 1e-6;
constexpr double kAmpD = 0.000000;
constexpr double kAmpIZone = 0;
constexpr double kAmpFF = 0.000015;

// constexpr double kAmpLowerP = 6e-5;
// constexpr double kAmpLowerI = 1e-6;
// constexpr double kAmpLowerD = 0;
// constexpr double kAmpLowerIZone = 0;
// constexpr double kAmpLowerFF = 0.000015;
// constexpr double kAmpLowerVelocity = 0.254;

// constexpr double kSubwooferUpperP = 6e-5;
// constexpr double kSubwooferUpperI = 1e-6;
// constexpr double kSubwooferUpperD = 0;
// constexpr double kSubwooferUpperIZone = 0;
// constexpr double kSubwooferUpperFF = 0.000015;
// constexpr double kSubwooferUpperVelocity = 0.75;

// constexpr double kSubwooferLowerP = 6e-5;
// constexpr double kSubwooferLowerI = 1e-6;
// constexpr double kSubwooferLowerD = 0;
// constexpr double kSubwooferLowerIZone = 0;
// constexpr double kSubwooferLowerFF = 0.000015;
// constexpr double kSubwooferLowerVelocity = 0.75;

const std::string kSpeakerUpperName = "Speaker Upper";
const std::string kSpeakerLowerName = "Speaker Lower";
const std::string kAmpUpperName = "Amp Upper";
const std::string kAmpLowerName = "Amp Lower";
const std::string kSubwooferUpperName = "Subwoofer Upper";
const std::string kSubwooferLowerName = "Subwoofer Lower";
}  // namespace ScoringPID
}  // namespace ScoringConstants