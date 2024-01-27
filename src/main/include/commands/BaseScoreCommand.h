#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ScoringSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class BaseScoreCommand : public frc2::CommandHelper<frc2::Command, BaseScoreCommand> {
public:
    explicit BaseScoreCommand(ScoringSubsystem* subsystem, IntakeSubsystem* intk, ScoringDirection direction)
        : m_score{subsystem}, m_intake{intk}, isFinished{false}, m_state{ScoreState::FlywheelRamp}, m_direction{direction} {
        AddRequirements({m_intake, m_score});
    }

    void Initialize() override {
        if (!m_intake->NotePresent()) {
            isFinished = true;
            return;
        }
        m_state = ScoreState::FlywheelRamp;
        m_score->StartScoringRamp(m_direction);
        isFinished = false;
    }

    void Execute() override {
    switch (m_state) {
      case ScoreState::FlywheelRamp:
        if (m_score->GetMotorAtScoringSpeed(m_direction)) {
          m_score->SpinVectorSide(m_direction);
          m_intake->In();
          m_state = ScoreState::Feeding;
        }
        break;
      case ScoreState::Feeding:
        if (!m_score->GetMotorFreeWheel(m_direction)) {
          m_state = ScoreState::Shooting;
        }
        break;
      case ScoreState::Shooting:
        if (m_score->GetMotorFreeWheel(m_direction)) {
          isFinished = true;
        }
        break;
      default:
        isFinished = true;
        break;
    }
  }

    void End(bool interrupted) override {
        m_intake->Stop();
        m_score->Stop();
    }

    bool IsFinished() override {
        return isFinished;
    }

protected:
    enum class ScoreState {
        FlywheelRamp,
        Feeding,
        Shooting,
    };

    ScoringSubsystem* m_score;
    IntakeSubsystem* m_intake;
    bool isFinished;
    ScoreState m_state;
    ScoringDirection m_direction;
};
