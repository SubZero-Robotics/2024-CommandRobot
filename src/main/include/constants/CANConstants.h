#pragma once

// Motor IDs
namespace CANConstants {
constexpr int kArmSpinnyBoiId = 62;
constexpr int kLeftIntakeSpinnyBoiId = 20;
constexpr int kVectorSpinnyBoiId = 22;
constexpr int kAmpLowerSpinnyBoiId = 24;
constexpr int kAmpUpperSpinnyBoiId = 21;
constexpr int kSpeakerLowerSpinnyBoiId = 25;
constexpr int kSpeakerUpperSpinnyBoiId = 19;
constexpr int kPigeonCanId1 = 9;
constexpr int kPigeonCanId2 = 13;

constexpr int kTicksPerMotorRotation = 42;
}  // namespace CANConstants