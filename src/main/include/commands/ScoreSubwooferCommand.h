#pragma once

#include "BaseScoreCommand.h"

class ScoreSubwoofer : public BaseScoreCommand {
 public:
  explicit ScoreSubwoofer(ScoringSubsystem* subsystem, IntakeSubsystem* intk)
      : BaseScoreCommand(subsystem, intk, ScoringDirection::Subwoofer) {}
};