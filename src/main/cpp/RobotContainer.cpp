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
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "autos/AutoFactory.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/InputUtils.h"
#include "utils/ShuffleboardLogger.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](std::vector<frc::Pose2d> poses) {
        m_drive.GetField()->GetObject("path")->SetPoses(poses);
      });

  pathplanner::PPHolonomicDriveController::setRotationTargetOverride(
      std::bind(&RobotContainer::GetRotationTargetOverride, this));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        InputUtils::DeadzoneAxes axes = InputUtils::CalculateCircularDeadzone(
            m_driverController.GetLeftX(), m_driverController.GetLeftY(),
            OIConstants::kDriveDeadband);

        TurnToPose* turnToPose = m_shouldAim ? &m_turnToPose : nullptr;

        m_drive.Drive(
            -units::meters_per_second_t{axes.y},
            -units::meters_per_second_t{axes.x},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true, kLoopTime, turnToPose);
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
      AutoConstants::kShootSubwooferName,
      ScoringCommands::ScoreShoot([] { return ScoringDirection::Subwoofer; },
                                  &m_scoring, &m_intake, &m_arm));
  pathplanner::NamedCommands::registerCommand(
      AutoConstants::kScoreSubwooferName,
      ScoringCommands::Score([] { return ScoringDirection::Subwoofer; },
                             &m_scoring, &m_intake, &m_arm));
  pathplanner::NamedCommands::registerCommand(
      "Spin up Speaker",
      ScoringCommands::ScoreRamp([] { return ScoringDirection::Subwoofer; },
                                 &m_scoring, &m_intake, &m_arm));
  pathplanner::NamedCommands::registerCommand(
      AutoConstants::kIntakeName,
      frc2::InstantCommand(
          [this] { ConsoleWriter.logVerbose("Autos", "Intaking %s", ""); })
          .ToPtr()
          .AndThen(IntakingCommands::Intake(&m_intake, &m_scoring)));
  using namespace AutoConstants;
  m_chooser.SetDefaultOption(AutoConstants::kDefaultAutoName,
                             AutoType::LeaveWing);
  m_chooser.AddOption("4 Note Auto", AutoType::FourNoteAuto);
  m_chooser.AddOption("Place and leave", AutoType::PlaceAndLeave);
  m_chooser.AddOption("3 Note Auto", AutoType::ThreeNoteAuto);
  m_chooser.AddOption("2 Note and Center Line", AutoType::TwoNoteAuto);
  m_chooser.AddOption("2 Note Center Note Under Stage",
                      AutoType::TwoNoteCenter);
  m_chooser.AddOption("2 Note Source Side", AutoType::TwoNoteSource);
  m_chooser.AddOption("3 Note Center Note 3 + 4", AutoType::ThreeNoteCenter);

  m_chooser.AddOption("Empty Auto", AutoType::EmptyAuto);

  ShuffleboardLogger::getInstance().logVerbose("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  m_driverController.Start().WhileTrue(new frc2::RunCommand(
      [this] {
        m_drive.SetX();
        // ConsoleWriter.logVerbose("Drive", "SetX %s", "");
      },
      {&m_drive}));

#ifndef TEST_SWERVE_BOT
  m_driverController.LeftTrigger(OIConstants::kDriveDeadband)
      .OnTrue(m_leds.Climbing().AndThen(m_leds.AmogusFace()))
      .WhileTrue(ExtendClimbCommand(
                     &m_leftClimb,
                     [this] { return m_driverController.GetLeftTriggerAxis(); },
                     [this] { return 0; })
                     .ToPtr());

  m_driverController.RightTrigger(OIConstants::kDriveDeadband)
      .OnTrue(m_leds.Climbing().AndThen(m_leds.AmogusFace()))
      .WhileTrue(
          ExtendClimbCommand(
              &m_rightClimb,
              [this] { return m_driverController.GetRightTriggerAxis(); },
              [this] { return 0; })
              .ToPtr());

  m_driverController.B().OnTrue(
      m_leds.Intaking()
          .AndThen(m_leds.AngryFace())
          .AndThen(IntakingCommands::Intake(&m_intake, &m_scoring))
          .AndThen((m_leds.Loaded().AndThen(m_leds.HappyFace())).Unless([this] {
            return !m_intake.NotePresent();
          })));

  m_driverController.X().OnTrue(
      m_leds.ScoringAmp()
          .AndThen(ScoringCommands::Score(
              [] { return ScoringDirection::PodiumSide; }, &m_scoring,
              &m_intake, &m_arm))
          .AndThen(m_leds.BlinkingFace())
          .AndThen(m_leds.Idling()));

  m_driverController.A().OnTrue(
      m_leds.ScoringAmp()
          .AndThen(
              ScoringCommands::Score([] { return ScoringDirection::AmpSide; },
                                     &m_scoring, &m_intake, &m_arm))
          .AndThen(m_leds.BlinkingFace())
          .AndThen(m_leds.Idling()));

  m_driverController.Y().OnTrue(
      m_leds.ScoringSubwoofer()
          .AndThen(
              ScoringCommands::Score([] { return ScoringDirection::Subwoofer; },
                                     &m_scoring, &m_intake, &m_arm))
          .AndThen(m_leds.BlinkingFace())
          .AndThen(m_leds.Idling()));

  m_driverController.LeftBumper()
      .OnTrue(m_leds.Climbing().AndThen(m_leds.AmogusFace()))
      .WhileTrue(
          ExtendClimbCommand(
              &m_leftClimb, [this] { return 0; },
              [this] { return ClimbConstants::kClimberExtendSpeed.value(); })
              .ToPtr())
      .WhileTrue(
          ExtendClimbCommand(
              &m_rightClimb, [this] { return 0; },
              [this] { return ClimbConstants::kClimberExtendSpeed.value(); })
              .ToPtr());

  m_driverController.RightBumper().WhileTrue(
      m_leds.Outaking().AndThen(IntakeOut(&m_intake, &m_scoring).ToPtr()));

  m_driverController.RightStick().OnTrue(
      frc2::InstantCommand([this] { ToggleAimbot(); }).ToPtr());

  m_driverController.LeftStick().OnTrue(tracker.IntakeTarget().AndThen(
      frc2::InstantCommand([this] {
        if (!m_state.m_active) {
          m_state.m_currentState = RobotState::ScoringSubwoofer;
          m_state.SetDesiredState();
        }
      }).ToPtr()));

  ConfigureAutoBindings();
#endif
#ifdef TEST_SWERVE_BOT
  m_driverController.A().OnTrue(m_leds.ScoringAmp()
                                    .WithTimeout(5_s)
                                    .AndThen(m_leds.Idling())
                                    .AndThen(frc2::InstantCommand([] {
                                               ConsoleWriter.logVerbose(
                                                   "'A' Button", "Pressed");
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
  // Maps to / on keyboard
  // m_operatorController.Button(0).OnTrue(
  // TODO: GOTO APPROX SCORING
  // );

  // Maps to NUM LOCK on keyboard
  m_operatorController.Button(3).OnTrue(
      // GOTO SRC
      frc2::InstantCommand([this] {
        if (!m_state.m_active) {
          m_state.m_currentState = RobotState::SourceCenter;
          m_state.SetDesiredState();
        }
      }).ToPtr());

  // Maps to 7 on keyboard
  m_operatorController.Button(4).OnTrue(frc2::InstantCommand([this] {
                                          if (!m_state.m_active) {
                                            m_state.m_currentState =
                                                RobotState::ScoringAmp;
                                            m_state.SetDesiredState();
                                          }
                                        }).ToPtr());

  // Maps to 8 on keyboard
  m_operatorController.Button(5).OnTrue(frc2::InstantCommand([this] {
                                          if (!m_state.m_active) {
                                            m_state.m_currentState =
                                                RobotState::ScoringSpeaker;
                                            m_state.SetDesiredState();
                                          }
                                        }).ToPtr());

  // Maps to 9 on keyboard
  m_operatorController.Button(6).OnTrue(frc2::InstantCommand([this] {
                                          if (!m_state.m_active) {
                                            m_state.m_currentState =
                                                RobotState::ScoringSubwoofer;
                                            m_state.SetDesiredState();
                                          }
                                        }).ToPtr());

  // Maps to 4 on keyboard
  m_operatorController.Button(7).OnTrue(frc2::InstantCommand([this] {
                                          if (!m_state.m_active) {
                                            m_state.m_currentState =
                                                RobotState::AutoSequenceAmp;
                                            m_state.SetDesiredState();
                                          }
                                        }).ToPtr());

  // Maps to 5 on keyboard
  m_operatorController.Button(8).OnTrue(
      frc2::InstantCommand([this] { m_drive.ZeroHeading(); }).ToPtr());

  // Maps to 6 on keyboard
  m_operatorController.Button(9).OnTrue(
      m_rightClimb.MoveToPositionAbsolute(18_in).AlongWith(
          m_leftClimb.MoveToPositionAbsolute(18_in)));

  // Maps to 1 on keyboard
  m_operatorController.Button(10).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::ClimbStageLeft;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  // Maps to 2 on keyboard
  m_operatorController.Button(16).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::ClimbStageCenter;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  // Maps to 3 on keyboard
  m_operatorController.Button(17).OnTrue(frc2::InstantCommand([this] {
                                           if (!m_state.m_active) {
                                             m_state.m_currentState =
                                                 RobotState::ClimbStageRight;
                                             m_state.SetDesiredState();
                                           }
                                         }).ToPtr());

  // Maps to 0 on keyboard
  m_operatorController.Button(18).OnTrue(
      m_leds.ScoringAmp()
          .AndThen(ScoringCommands::Score(
              [] { return ScoringDirection::FeedPodium; }, &m_scoring,
              &m_intake, &m_arm))
          .AndThen(m_leds.BlinkingFace())
          .AndThen(m_leds.Idling()));

  // Maps to DEL on keyboard
  m_operatorController.Button(18).OnTrue(tracker.IntakeTarget());
}
#endif

frc2::Command* RobotContainer::GetAutonomousCommand() {
  auto autoType = m_chooser.GetSelected();
  autoCommand = AutoFactory::GetAuto(autoType);
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

void RobotContainer::DisableSubsystems() {
  m_arm.DisablePid();
  m_leftClimb.DisablePid();
  m_rightClimb.DisablePid();
}

void RobotContainer::Initialize() {
  m_arm.OnInit();
  m_leftClimb.OnInit();
  m_rightClimb.OnInit();
}

void RobotContainer::StopMotors() {
  m_intake.Stop();
  m_scoring.Stop();
}

void RobotContainer::Periodic() {
  frc::SmartDashboard::PutData("Robot2d", &m_mech);

  frc::SmartDashboard::PutBoolean("TURN TO POSE AT GOAL",
                                  m_turnToPose.AtGoal());

  m_turnToPose.Update();
  auto targets = tracker.GetTargets();

  // ! This causes spam! Better way of tracking + updating individual targets?
  // for (auto& target : targets) {
  //   std::string label =
  //       "tracked_gamepiece [" + std::to_string(target.confidence) + "]";
  //   auto pose = tracker.GetTargetPose(target);

  //   if (pose) m_drive.GetField()->GetObject(label)->SetPose(pose.value());
  // }

  if (m_intake.NotePresent()) {
    // Note is present, get ready to score it
    auto locations = DrivingCommands::GetSortedLocations(m_drive.GetPose());
    bool inRange = false;

    for (auto& location : locations) {
      if (location.hypotDistance <= location.locationRadius) {
        inRange = m_aimbotEnabled && true;
        m_turnToPose.SetTargetPose(location.trackedPose);

        frc::SmartDashboard::PutNumber(
            "TURN TO POSE TARGET deg",
            location.trackedPose.Rotation().Degrees().value());

        break;
      }
    }

    m_shouldAim = m_aimbotEnabled && inRange;
    } else {
      // auto targetPose = tracker.GetBestTargetPose(targets);

      // frc::SmartDashboard::PutBoolean("HAS TARGET LOCK",
      //                                 tracker.HasTargetLock(targets));

      // if (targetPose) {
      //   m_shouldAim = m_aimbotEnabled;
      //   m_turnToPose.SetTargetPose(targetPose.value());

      //   frc::SmartDashboard::PutNumber(
      //       "TURN TO POSE TARGET deg",
      //       targetPose.value().Rotation().Degrees().value());
      // } else {
      //   m_shouldAim = false;
      // }
      m_shouldAim = false;
  }
}

std::optional<frc::Rotation2d> RobotContainer::GetRotationTargetOverride() {
  auto targetPose = m_turnToPose.GetTargetPose();

  if (m_autoAcquiringNote && targetPose) {
    return TurnToPose::GetAngleFromOtherPose(m_drive.GetPose(),
                                             targetPose.value());
  }

  return std::nullopt;
}

void RobotContainer::ToggleAimbot() {
  m_aimbotEnabled = !m_aimbotEnabled;
  frc::SmartDashboard::PutBoolean("aimbot enabled", m_aimbotEnabled);
}