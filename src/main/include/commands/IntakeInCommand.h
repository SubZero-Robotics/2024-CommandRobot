#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

class IntakeIn : public frc2::CommandHelper<frc2::Command, IntakeIn> {
   public:
    /**
     * Creates a new Intake.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit IntakeIn(IntakeSubsystem* subsystem)
        : m_intake{subsystem}, isFinished{false} {
        // Register that this command requires the subsystem.
        AddRequirements(m_intake);
    }

    void Execute() override { m_intake->In(); }

    bool IsFinished() override { return isFinished; }

    void End(bool interrupted) { m_intake->Stop(); }

   private:
    IntakeSubsystem* m_intake;
    bool isFinished;
};