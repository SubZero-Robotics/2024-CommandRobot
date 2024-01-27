#pragma once

#include <math.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Constants.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"

// For sim to work
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class BalanceCommand : public frc2::CommandHelper<frc2::Command, BalanceCommand> {
    public:
        BalanceCommand(DriveSubsystem *driveSubsystem, ClimbSubsystem *leftClimb, ClimbSubsystem *rightClimb)
        : m_drive(driveSubsystem), m_leftClimb(leftClimb), m_rightClimb(rightClimb) {
            AddRequirements({m_drive, m_leftClimb, m_rightClimb});
        }

        void Initialize() override {
            isFinished = false;
            double roll = m_drive->getGyro()->GetRoll() * (M_PI / 180);
            double climberExtentDistance = sin(roll) * ClimbConstants::kClimberOffsetDistance.value();

            if (climberExtentDistance < 0) {
                m_rightClimb->MoveRelative(climberExtentDistance);
            } else {
                m_leftClimb->MoveRelative(-climberExtentDistance);
            }
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
        DriveSubsystem *m_drive;
        ClimbSubsystem *m_leftClimb;
        ClimbSubsystem *m_rightClimb;
};