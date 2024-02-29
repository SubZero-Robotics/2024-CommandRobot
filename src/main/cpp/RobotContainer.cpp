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
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/CommandUtils.h"
#include "utils/InputUtils.h"
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
        InputUtils::DeadzoneAxes axes = InputUtils::CalculateCircularDeadzone(
            m_driverController.GetLeftX(), m_driverController.GetLeftY(),
            OIConstants::kDriveDeadband);
        m_drive.Drive(
            -units::meters_per_second_t{axes.y},
            -units::meters_per_second_t{axes.x},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true, kLoopTime);
      },
      {&m_drive}));
#ifndef TEST_SWERVE_BOT
  RegisterAutos();
#endif
}

void RobotContainer::RegisterAutos() {
  pathplanner::NamedCommands::registerCommand(AutoConstants::kLedFunniName,
                                              m_leds.Intaking());
  pathplanner::NamedCommands::registerCommand(
      AutoConstants::kScoreSubwooferName,
      ScoringCommands::Score([] { return ScoringDirection::Subwoofer; },
                             &m_scoring, &m_intake));
  pathplanner::NamedCommands::registerCommand(
      AutoConstants::kIntakeName,
      frc2::InstantCommand([this] {
        ConsoleLogger::getInstance().logVerbose("Autos", "Intaking %s", "");
      })
          .ToPtr()
          .AndThen(IntakingCommands::Intake(&m_intake, &m_scoring)));

  m_chooser.SetDefaultOption(AutoConstants::kDefaultAutoName,
                             AutoConstants::kDefaultAutoName);
  // m_chooser.AddOption("3 in Amp", "3 in Amp");
  // m_chooser.AddOption("2 Note Auto", "2 Note Auto");
  m_chooser.AddOption("4 Note Auto", "4 Note Auto");
  m_chooser.AddOption("Place and leave", "Place and leave");

  ShuffleboardLogger::getInstance().logVerbose("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  m_driverController.Start().WhileTrue(new frc2::RunCommand(
      [this] {
        m_drive.SetX();
        // ConsoleLogger::getInstance().logVerbose("Drive", "SetX %s", "");
      },
      {&m_drive}));

#ifndef TEST_SWERVE_BOT
  m_driverController.LeftTrigger(OIConstants::kDriveDeadband)
      .WhileTrue(m_leds.Climbing().AndThen(
          ExtendClimbCommand(
              &m_leftClimb,
              [this] { return m_driverController.GetLeftTriggerAxis(); },
              [this] { return 0; })
              .ToPtr()));

  m_driverController.RightTrigger(OIConstants::kDriveDeadband)
      .WhileTrue(m_leds.Climbing().AndThen(
          ExtendClimbCommand(
              &m_rightClimb,
              [this] { return m_driverController.GetRightTriggerAxis(); },
              [this] { return 0; })
              .ToPtr()));

  m_driverController.B().OnTrue(
      // IntakingCommands::Intake(&m_intake, &m_scoring));
      m_leds.Intaking()
          .AndThen(IntakingCommands::Intake(&m_intake, &m_scoring))
          .AndThen(m_leds.Loaded()));

  m_driverController.X().OnTrue(IntakingCommands::OuttakeUntilPresent(
      &m_intake, &m_scoring, ScoringDirection::SpeakerSide));

  m_driverController.A().OnTrue(
      m_leds.ScoringAmp().AndThen(ScoringCommands::Score(
          [] { return ScoringDirection::AmpSide; }, &m_scoring, &m_intake)));

  m_driverController.Y().OnTrue(
      m_leds.ScoringSubwoofer().AndThen(ScoringCommands::Score(
          [] { return ScoringDirection::Subwoofer; }, &m_scoring, &m_intake)));

  m_driverController.LeftBumper()
      .WhileTrue(m_leds.Climbing().AndThen(
          ExtendClimbCommand(
              &m_leftClimb, [this] { return 0; },
              [this] { return ClimbConstants::kCLimberExtendSpeed; })
              .ToPtr()))
      .WhileTrue(m_leds.Climbing().AndThen(
          ExtendClimbCommand(
              &m_rightClimb, [this] { return 0; },
              [this] { return ClimbConstants::kCLimberExtendSpeed; })
              .ToPtr()));

  m_driverController.RightBumper().WhileTrue(
      m_leds.Outaking().AndThen(IntakeOut(&m_intake, &m_scoring).ToPtr()));

  m_driverController.RightStick().OnTrue(
      DrivingCommands::SnapToAngle(&m_drive));

  ConfigureAutoBindings();
#endif
#ifdef TEST_SWERVE_BOT
  m_driverController.A().OnTrue(
      m_leds.ScoringAmp()
          .WithTimeout(5_s)
          .AndThen(m_leds.Idling())
          .AndThen(frc2::InstantCommand([] {
                     ConsoleLogger::getInstance().logVerbose("'A' Button",
                                                             "Pressed");
                   }).ToPtr()));
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
  m_operatorController.A().OnTrue(frc2::InstantCommand([this] {
                                    if (!m_state.m_active) {
                                      m_state.m_currentState =
                                          RobotState::ScoringSpeaker;
                                      m_state.SetDesiredState();
                                    }
                                  }).ToPtr());

  // Maps to 8 on keyboard
  m_operatorController.B().OnTrue(frc2::InstantCommand([this] {
                                    if (!m_state.m_active) {
                                      m_state.m_currentState =
                                          RobotState::ScoringSubwoofer;
                                      m_state.SetDesiredState();
                                    }
                                  }).ToPtr());

  // Maps to 7 on keyboard
  m_operatorController.X().OnTrue(frc2::InstantCommand([this] {
                                    if (!m_state.m_active) {
                                      m_state.m_currentState =
                                          RobotState::ScoringAmp;
                                      m_state.SetDesiredState();
                                    }
                                  }).ToPtr());

  // Maps to 4 on keyboard
  m_operatorController.Y().OnTrue(frc2::InstantCommand([this] {
                                    if (!m_state.m_active) {
                                      m_state.m_currentState =
                                          RobotState::AutoSequenceAmp;
                                      m_state.SetDesiredState();
                                    }
                                  }).ToPtr());

  // Maps to 1 on keyboard
  m_operatorController.LeftBumper().OnTrue(frc2::InstantCommand([this] {
                                             if (!m_state.m_active) {
                                               m_state.m_currentState =
                                                   RobotState::ClimbStageLeft;
                                               m_state.SetDesiredState();
                                             }
                                           }).ToPtr());

  // Maps to 2 on keyboard
  m_operatorController.RightBumper().OnTrue(
      frc2::InstantCommand([this] {
        if (!m_state.m_active) {
          m_state.m_currentState = RobotState::ClimbStageCenter;
          m_state.SetDesiredState();
        }
      }).ToPtr());

  // Maps to 3 on keyboard
  m_operatorController.LeftStick().OnTrue(frc2::InstantCommand([this] {
                                            if (!m_state.m_active) {
                                              m_state.m_currentState =
                                                  RobotState::ClimbStageRight;
                                              m_state.SetDesiredState();
                                            }
                                          }).ToPtr());

  m_operatorController.Button(15).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::SourceLeft;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  m_operatorController.Button(16).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::SourceCenter;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  m_operatorController.Button(17).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::SourceRight;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  m_operatorController.Button(18).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::Funni;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  m_operatorController.Button(20).OnTrue(
      frc2::InstantCommand([this] { m_drive.ZeroHeading(); }).ToPtr());
}
#endif

frc2::Command* RobotContainer::GetAutonomousCommand() {
  std::string autoName = m_chooser.GetSelected();
  autoCommand = PathPlannerAuto(autoName).ToPtr();
  return autoCommand.get();
}

void RobotContainer::ClearCurrentStateCommand() {
  m_state.m_active = false;
  // m_leds.ErrorAsync();
}

void RobotContainer::StartIdling() {
  m_state.m_active = false;
  m_leds.IdlingAsync();
}

void RobotContainer::ResetPose() {
  m_drive.ResetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}