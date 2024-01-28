#pragma once

#include "subsystems/DriveSubsystem.h"

#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

class DriveVelocity : public frc2::CommandHelper<frc2::Command, DriveVelocity> {
    public:
        DriveVelocity(units::meter_t scalar, units::degree_t angle, units::meters_per_second_t speed, DriveSubsystem* drive) {}

        void Initialize() override;

        void Execute() override;

        void End(bool interrupted) override;
};