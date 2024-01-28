#pragma once

#include "subsystems/DriveSubsystem.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include <math.h>

class DriveVelocity : public frc2::CommandHelper<frc2::Command, DriveVelocity> {
    public:
        DriveVelocity(units::degree_t angle, units::meters_per_second_t speed,
            DriveSubsystem* drive) : m_angle(angle), m_velocity(speed), m_drive(drive) {
            AddRequirements(drive);
        }

        void Execute() override {
            auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
                m_velocity * cos(units::radian_t(m_angle).value()),         // vx where positive is towards the robot's front
                m_velocity * sin(units::radian_t(m_angle).value()),         // vy where positive is towards the robot's left
                AutoConstants::kMaxAngularSpeed,
                DriveConstants::kLoopTime
            );
            m_drive->Drive(chassisSpeeds);
        }

        void End(bool interrupted) override {
            auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
                0_mps,
                0_mps,
                AutoConstants::kMaxAngularSpeed,
                DriveConstants::kLoopTime
            );
            m_drive->Drive(chassisSpeeds);
        }

    private:
        units::degree_t m_angle;
        units::meters_per_second_t m_velocity;
        DriveSubsystem *m_drive;
};