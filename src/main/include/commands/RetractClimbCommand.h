#pragma once

#pragma once

#include <math.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Constants.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"

class RetractClimbCommand : public frc2::CommandHelper<frc2::Command, RetractClimbCommand> {
    public:
        RetractClimbCommand(ClimbSubsystem *leftClimb, ClimbSubsystem *rightClimb)
        : m_leftClimb(leftClimb), m_rightClimb(rightClimb) {}

        void Initialize() override {
            isFinished = false;
            m_rightClimb->MoveToPosition(ClimbConstants::kClimbRetractPosition);
            m_leftClimb->MoveToPosition(ClimbConstants::kClimbRetractPosition);
        }

        void Execute() override {
            isFinished = !(m_leftClimb->GetIsMovingToPosition() || m_rightClimb->GetIsMovingToPosition());
        }

        bool IsFinished() override {
            return isFinished;
        }

        void End(bool interrupted) override {
            m_leftClimb->StopMovement();
            m_rightClimb->StopMovement();
        }
    
    private:
        bool isFinished;
        ClimbSubsystem *m_leftClimb;
        ClimbSubsystem *m_rightClimb;
};