#pragma once

#include "BaseScoreCommand.h"

class ScoreSpeaker : public BaseScoreCommand {
 public:
  explicit ScoreSpeaker(ScoringSubsystem* subsystem, IntakeSubsystem* intk)
      : BaseScoreCommand(subsystem, intk, ScoringDirection::SpeakerSide) {}
};