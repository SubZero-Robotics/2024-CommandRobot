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
  m_chooser.SetDefaultOption("Leave Community", m_defaultAuto.get());
  ShuffleboardLogger::getInstance().logVerbose("Auto Modes", &m_chooser);

  // TODO: replace with a FUNNI animation
  pathplanner::NamedCommands::registerCommand("LedFunni", m_leds.Intaking());
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

//   frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
//       .OnTrue(pathplanner::AutoBuilder::pathfindToPose(
//           frc::Pose2d{1.5_m, 5.5_m, 0_rad},
//           pathplanner::PathConstraints{3.0_mps, 4.0_mps_sq, 540_deg_per_s,
//                                        720_deg_per_s_sq},
//           0.0_mps,  // Goal end velocity in meters/sec
//           0.0_m     // Rotation delay distance in meters. This is how far
//                     // the robot should travel before attempting to rotate.
//           ));

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

  ConfigureAutoBindings();
#endif
#ifdef TEST_SWERVE_BOT
  m_driverController.A().OnTrue(
      m_leds.ScoringAmp().WithTimeout(5_s).AndThen(m_leds.Idling()));
  m_driverController.B().OnTrue(
      m_leds.Intaking().WithTimeout(5_s).AndThen(m_leds.Idling()));
  m_driverController.X().OnTrue(
      m_leds.ScoringSpeaker().WithTimeout(5_s).AndThen(m_leds.Idling()));
  m_driverController.Y().OnTrue(
      m_leds.Loaded().WithTimeout(5_s).AndThen(m_leds.Idling()));
  m_driverController.LeftBumper().OnTrue(
      m_leds.Climbing().WithTimeout(5_s).AndThen(m_leds.Idling()));
  m_driverController.RightBumper().OnTrue(
      m_leds.Error().WithTimeout(5_s).AndThen(m_leds.Idling()));
#endif
}

#ifndef TEST_SWERVE_BOT
void RobotContainer::ConfigureAutoBindings() {
  // Maps to 9 on keyboard
  m_operatorController.A().OnTrue(
      m_state.SetState(RobotState::ScoringSpeaker)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 8 on keyboard
  m_operatorController.B().OnTrue(
      m_state.SetState(RobotState::ScoringSubwoofer)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 7 on keyboard
  m_operatorController.X().OnTrue(
      m_state.SetState(RobotState::ScoringAmp)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 4 on keyboard
  m_operatorController.Y().OnTrue(
      m_state.SetState(RobotState::Intaking)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 1 on keyboard
  m_operatorController.LeftBumper().OnTrue(
      m_state.SetState(RobotState::ClimbStageLeft)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 2 on keyboard
  m_operatorController.RightBumper().OnTrue(
      m_state.SetState(RobotState::ClimbStageCenter)
          .AndThen(m_state.RunStateDeferred().ToPtr()));

  // Maps to 3 on keyboard
  m_operatorController.LeftStick().OnTrue(
      m_state.SetState(RobotState::ClimbStageRight)
          .AndThen(m_state.RunStateDeferred().ToPtr()));
}
#endif

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_chooser.GetSelected();
}