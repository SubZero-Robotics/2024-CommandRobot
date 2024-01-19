#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ShooterShoot : public frc2::CommandHelper<frc2::Command, ShooterShoot> {
   public:
    /**
     * Creates a new Intake.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit ShooterShoot(ShooterSubsystem* subsystem)
        : m_shooter{subsystem}, isFinished{false} {
        // Register that this command requires the subsystem.
        AddRequirements(m_shooter);
    }

    void Execute() override { m_shooter->Out(); }

    bool IsFinished() override { return isFinished; }

    void End(bool interrupted) { m_shooter->Stop(); }

   private:
    ShooterSubsystem* m_shooter;
    bool isFinished;
};