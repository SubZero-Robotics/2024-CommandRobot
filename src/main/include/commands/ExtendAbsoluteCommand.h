#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Constants.h"
#include "subsystems/ClimbSubsystem.h"

class ExtendAbsolute : public frc2::CommandHelper<frc2::Command, ExtendAbsolute> {
    public:
        ExtendAbsolute(ClimbSubsystem *climbLeft, ClimbSubsystem *climbRight) 
        : m_leftClimbSubsystem(climbLeft), m_rightClimbSubsystem(climbRight) {
            AddRequirements({m_leftClimbSubsystem, m_rightClimbSubsystem});
        }

        void Initialize() override {
            m_leftClimbSubsystem->MoveToPosition(ClimbConstants::kClimbExtensionPosition);
            m_rightClimbSubsystem->MoveToPosition(ClimbConstants::kClimbExtensionPosition);
            isFinished = false;
        }

        void Execute() override {
            isFinished = !m_leftClimbSubsystem->GetIsMovingToPosition() && !m_rightClimbSubsystem->GetIsMovingToPosition();
        }

        bool IsFinished() {
            return isFinished;
        }

        void End(bool interrupted) override {
            m_leftClimbSubsystem->StopMovement();
            m_rightClimbSubsystem->StopMovement();
        }

    private:
        bool isFinished;
        ClimbSubsystem *m_leftClimbSubsystem;
        ClimbSubsystem *m_rightClimbSubsystem;
};