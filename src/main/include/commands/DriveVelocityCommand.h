#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <math.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "subsystems/DriveSubsystem.h"

class DriveVelocity : public frc2::CommandHelper<frc2::Command, DriveVelocity> {
 public:
  DriveVelocity(units::degree_t angle, units::meters_per_second_t speed,
                DriveSubsystem* drive)
      : m_angle(angle), m_velocity(speed), m_drive(drive) {
    AddRequirements(drive);
  }

  void Execute() override {
    auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
        m_velocity *
            cos(units::radian_t(m_angle).value()),  // vx where positive is
                                                    // towards the robot's front
        0_mps,  // vy where positive is towards the robot's left
        0_rad_per_s, DriveConstants::kLoopTime);
    m_drive->Drive(chassisSpeeds);
  }

  void End(bool interrupted) override {
    auto chassisSpeeds = frc::ChassisSpeeds::Discretize(
        0_mps, 0_mps, AutoConstants::kMaxAngularSpeed,
        DriveConstants::kLoopTime);
    m_drive->Drive(chassisSpeeds);
  }

 private:
  units::degree_t m_angle;
  units::meters_per_second_t m_velocity;
  DriveSubsystem* m_drive;
};