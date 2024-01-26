#pragma once

#include "BaseScoreCommand.h"

class ScoreSpeaker : public BaseScoreCommand {
 public:
  explicit ScoreSpeaker(ScoringSubsystem* subsystem, IntakeSubsystem* intk)
      : BaseScoreCommand(subsystem, intk, ScoringDirection::SpeakerSide) {}

  void Execute() override {
    switch (m_state) {
      case ScoreState::FlywheelRamp:
        if (m_score->GetMotorAtScoringSpeed(ScoringDirection::SpeakerSide)) {
          m_score->SpinVectorSide(ScoringDirection::SpeakerSide);
          m_intake->In();
          m_state = ScoreState::Feeding;
        }
        break;
      case ScoreState::Feeding:
        if (!m_score->GetMotorFreeWheel(ScoringDirection::SpeakerSide)) {
          m_state = ScoreState::Shooting;
        }
        break;
      case ScoreState::Shooting:
        if (m_score->GetMotorFreeWheel(ScoringDirection::SpeakerSide)) {
          isFinished = true;
        }
        break;
      default:
        isFinished = true;
        break;
    }
  }
};