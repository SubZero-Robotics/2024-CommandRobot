// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/IterativeRobotBase.h>
#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "commands/BalanceCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "commands/ScoreAmpCommand.h"
#include "commands/ScoreSpeakerCommand.h"
#include "commands/ScoreSubwooferCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/ShuffleboardLogger.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, false, kLoopTime);
      },
      {&m_drive}));

  // This won't work since we're getting the reference of an r-value which goes
  // out of scope at the end of the method
  m_chooser.SetDefaultOption(
      "Leave Community",
      pathplanner::PathPlannerAuto(AutoConstants::kDefaultAutoName)
          .ToPtr()
          .Unwrap()
          .get());
  ShuffleboardLogger::getInstance().logVerbose("Auto Modes", &m_chooser);

    // TODO: replace with a FUNNI animation
  pathplanner::NamedCommands::registerCommand("LedFunni", m_leds.Intaking());
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
      .OnTrue(m_leds.GetDeferredFromState([this] { m_state.IncrementState(); return m_state.GetState(); }).ToPtr());

  frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
      .OnTrue(pathplanner::AutoBuilder::pathfindToPose(
          frc::Pose2d{1.5_m, 5.5_m, 0_rad},
          pathplanner::PathConstraints{3.0_mps, 4.0_mps_sq, 540_deg_per_s,
                                       720_deg_per_s_sq},
          0.0_mps,  // Goal end velocity in meters/sec
          0.0_m     // Rotation delay distance in meters. This is how far
                    // the robot should travel before attempting to rotate.
          ));

#ifndef TEST_SWERVE_BOT
  m_driverController.LeftTrigger(OIConstants::kDriveDeadband)
      .WhileTrue(
          ExtendClimbCommand(
              &m_leftClimb,
              [this] { return -m_driverController.GetLeftTriggerAxis(); },
              [this] { return 0; })
              .ToPtr());

  m_driverController.RightTrigger(OIConstants::kDriveDeadband)
      .WhileTrue(
          ExtendClimbCommand(
              &m_rightClimb,
              [this] { return -m_driverController.GetRightTriggerAxis(); },
              [this] { return 0; })
              .ToPtr());

  m_driverController.B().WhileTrue(IntakeIn(&m_intake).ToPtr());

  m_driverController.X().WhileTrue(ScoreSpeaker(&m_scoring, &m_intake).ToPtr());

  m_driverController.A().WhileTrue(ScoreAmp(&m_scoring, &m_intake).ToPtr());

  m_driverController.Y().WhileTrue(
      ScoreSubwoofer(&m_scoring, &m_intake).ToPtr());

  m_driverController.LeftBumper()
      .WhileTrue(ExtendClimbCommand(
                     &m_leftClimb, [this] { return 0; }, [this] { return 1; })
                     .ToPtr())
      .WhileTrue(ExtendClimbCommand(
                     &m_rightClimb, [this] { return 0; }, [this] { return 1; })
                     .ToPtr());

  m_driverController.RightBumper().WhileTrue(
      BalanceCommand(&m_drive, &m_leftClimb, &m_rightClimb).ToPtr());
#endif
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}