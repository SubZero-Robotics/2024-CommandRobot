#pragma once

#include "BaseScoreCommand.h"

class ScoreAmp : public BaseScoreCommand {
 public:
  explicit ScoreAmp(ScoringSubsystem* subsystem, IntakeSubsystem* intk)
      : BaseScoreCommand(subsystem, intk, ScoringDirection::AmpSide) {}
};