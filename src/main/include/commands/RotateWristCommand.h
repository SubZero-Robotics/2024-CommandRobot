#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/WristSubsystem.h"

class RotateWrist : public frc2::CommandHelper<frc2::Command, RotateWrist> {
   public:
    /**
     * Creates a new RotateWrist.
     *
     * @param subsystem The subsystem used by this command.
     */
    explicit RotateWrist(WristSubsystem* subsystem,
                         std::function<double()> rotation)
        : m_effector{subsystem}, m_rotation{rotation} {
        // Register that this command requires the subsystem.
        AddRequirements(m_effector);
    }

    void Execute() override {
        double rotation = m_rotation();

        if (abs(rotation) < OIConstants::kDriveDeadband) rotation = 0.0;

        m_effector->RunMotorExternal(rotation);
    }

   private:
    WristSubsystem* m_effector;
    std::function<double()> m_rotation;
};