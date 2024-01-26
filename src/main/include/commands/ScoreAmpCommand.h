#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ScoringSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class ScoreAmp : public frc2::CommandHelper<frc2::Command, ScoreAmp> {
    public:

        enum class ScoreState {
            FlywheelRamp,
            Feeding,
            Shooting,
        };

        explicit ScoreAmp(ScoringSubsystem* subsystem, IntakeSubsystem* intk) 
            : m_score{subsystem}, m_intake{intk}, isFinished{false} {
                AddRequirements({m_intake, m_score});
            }

        void Initialize() override {
            if (!m_intake->NotePresent()) {
                isFinished = true;
                return;
            }
            m_state = ScoreState::FlywheelRamp;
            m_score->StartScoringRamp(ScoringDirection::AmpSide);
            isFinished = false;
        }

        void Execute() override {
            // TODO: make not suck
            if (m_state == ScoreState::FlywheelRamp) {
                if (m_score->GetMotorAtScoringSpeed(ScoringDirection::AmpSide)) {
                    m_score->SpinVectorSide(ScoringDirection::AmpSide);
                    m_intake->In();
                    m_state = ScoreState::Feeding;
                }
            } else if (m_state == ScoreState::Feeding) {
                if (!m_score->GetMotorFreeWheel(ScoringDirection::AmpSide)) {
                    m_state = ScoreState::Shooting;
                }
            } else if (m_state == ScoreState::Shooting) {
                if (m_score->GetMotorFreeWheel(ScoringDirection::AmpSide)) {
                    isFinished = true;
                }
            } else {
                isFinished = true;
            }
        }

        void End(bool interrupted) override {
            m_intake->Stop();
            m_score->Stop();
        }

        bool IsFinished() override {
            return isFinished;
        }

    private:
        ScoringSubsystem* m_score;
        IntakeSubsystem* m_intake;
        bool isFinished;
        ScoreState m_state;
};