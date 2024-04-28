#pragma once

#include <functional>
#include <string>

namespace RobotConstants {
#ifdef TEST_SWERVE_BOT
const std::string kRoborioSerialNumber = "032B4B68";
#else
const std::string kRoborioSerialNumber = "0326F2F2";
#endif
}  // namespace RobotConstants

enum class RobotState {
  Manual = 0,
  ScoringSpeaker,
  ScoringAmp,
  ScoringSubwoofer,
  Loaded,
  Intaking,
  ClimbStageLeft,
  ClimbStageCenter,
  ClimbStageRight,
  SourceLeft,
  SourceCenter,
  SourceRight,
  Funni,
  AutoSequenceSpeaker,
  AutoSequenceAmp,
  AutoSequenceSubwoofer,
  EnemyWing,
  AllianceWing,
};

typedef std::function<RobotState()> StateGetter;