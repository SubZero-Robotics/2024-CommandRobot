#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ScoringSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class BaseScoreCommand : public frc2::CommandHelper<frc2::Command, BaseScoreCommand> {
public:
    explicit BaseScoreCommand(ScoringSubsystem* subsystem, IntakeSubsystem* intk, ScoringDirection direction)
        : m_score{subsystem}, m_intake{intk}, m_direction{direction}, isFinished{false} {
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
