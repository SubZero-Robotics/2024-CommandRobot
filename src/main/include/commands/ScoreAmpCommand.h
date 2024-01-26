#pragma once

#include "BaseScoreCommand.h"

class ScoreAmp : public BaseScoreCommand {
 public:
  explicit ScoreAmp(ScoringSubsystem* subsystem, IntakeSubsystem* intk)
      : BaseScoreCommand(subsystem, intk, ScoringDirection::AmpSide) {}

  void Execute() override {
    switch (m_state) {
      case ScoreState::FlywheelRamp:
        if (m_score->GetMotorAtScoringSpeed(ScoringDirection::AmpSide)) {
          m_score->SpinVectorSide(ScoringDirection::AmpSide);
          m_intake->In();
          m_state = ScoreState::Feeding;
        }
        break;
      case ScoreState::Feeding:
        if (!m_score->GetMotorFreeWheel(ScoringDirection::AmpSide)) {
          m_state = ScoreState::Shooting;
        }
        break;
      case ScoreState::Shooting:
        if (m_score->GetMotorFreeWheel(ScoringDirection::AmpSide)) {
          isFinished = true;
        }
        break;
      default:
        isFinished = true;
        break;
    }
  }
};