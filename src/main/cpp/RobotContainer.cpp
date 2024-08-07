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
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/pathfinding/RemoteADStar.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <subzero/logging/ShuffleboardLogger.h>
#include <subzero/utils/InputUtils.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <subzero/autonomous/AutoFactory.cpp>
#include <subzero/frc/smartdashboard/TaggedChooser.cpp>
#include <subzero/singleaxis/LinearSingleAxisSubsystem.cpp>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.cpp>
#include <utility>

#include "Constants.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/Funni.h"
#include "commands/IntakeOutCommand.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  pathplanner::PathPlannerLogging::setLogActivePathCallback(
      [this](std::vector<frc::Pose2d> poses) {
        m_drive.GetField()->GetObject("path")->SetPoses(poses);
      });

  // This causes a bug in PP :(
  // pathplanner::PPHolonomicDriveController::setRotationTargetOverride(
  //     std::bind(&RobotContainer::GetRotationTargetOverride, this));

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.

  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        InputUtils::DeadzoneAxes axes = InputUtils::CalculateCircularDeadzone(
            m_driverController.GetLeftX(), m_driverController.GetLeftY(),
            OIConstants::kDriveDeadband);

        ITurnToTarget* turnToTarget = m_shouldAim ? &m_turnToPose : nullptr;

        m_drive.Drive(
            -units::meters_per_second_t{axes.y},
            -units::meters_per_second_t{axes.x},
            -units::radians_per_second_t{frc::ApplyDeadband(
                m_driverController.GetRightX(), OIConstants::kDriveDeadband)},
            true, true, kLoopTime, turnToTarget);
      },
      {&m_drive}));
#ifndef TEST_SWERVE_BOT
  RegisterAutos();
#endif
}

void RobotContainer::RegisterAutos() {
#ifdef USE_REMOTE_AD_STAR
  pathplanner::Pathfinding::setPathfinder(
      std::make_unique<pathplanner::RemoteADStar>());
#endif

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

  m_autoChooser.Initialize();
  m_ignoreLimitChooser.SetDefaultOption("Don't Ignore", false);
  m_ignoreLimitChooser.AddOption("Ignore", true);
  frc::SmartDashboard::PutData("Climb Limits", &m_ignoreLimitChooser);
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
#ifndef USING_SYSID
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
#elif USING_SYSID
  m_driverController.A().OnTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kForward));

  m_driverController.B().OnTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  m_driverController.X().OnTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kForward));

  m_driverController.Y().OnTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kReverse));
#endif

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

  m_driverController.LeftStick().OnTrue(
      m_leds.ScoringSpeaker()
          .AndThen(ScoringCommands::Score(
              [] { return ScoringDirection::FeedPodium; }, &m_scoring,
              &m_intake, &m_arm))
          .AndThen(m_leds.BlinkingFace())
          .AndThen(m_leds.Idling()));

  m_driverController
      .POVUp(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop())
      .CastTo<frc2::Trigger>()
      .OnTrue(
          m_rightClimb
              .MoveToPositionAbsolute(ClimbConstants::kClimbExtensionPosition)
              .AlongWith(m_leftClimb.MoveToPositionAbsolute(
                  ClimbConstants::kClimbExtensionPosition)));

  m_driverController
      .POVDown(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop())
      .CastTo<frc2::Trigger>()
      .OnTrue(frc2::InstantCommand([this] { ToggleAutoScoring(); }).ToPtr());

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
  m_operatorController.Button(7).OnTrue(
      m_leds.OwOFace()
          .AndThen(m_leds.Error())
          .AndThen(m_rightClimb.MoveToPositionAbsolute(10_in).AlongWith(
              m_leftClimb.MoveToPositionAbsolute(10_in)))
          .AndThen(m_arm.MoveToPositionAbsolute(ArmConstants::kAmpRotation))
          .AndThen(frc2::WaitCommand(8_s).ToPtr())
          .AndThen(m_rightClimb.MoveToPositionAbsolute(1_in).AlongWith(
              m_leftClimb.MoveToPositionAbsolute(1_in)))
          .AndThen(m_arm.MoveToPositionAbsolute(ArmConstants::kHomeRotation))
          .AndThen(m_leds.Idling()));

  // .OnTrue(frc2::InstantCommand([this] {
  //                                         if (!m_state.m_active) {
  //                                           m_state.m_currentState =
  //                                               RobotState::AutoSequenceAmp;
  //                                           m_state.SetDesiredState();
  //                                         }
  //                                       }).ToPtr());

  // Maps to 5 on keyboard
  m_operatorController.Button(8).OnTrue(
      frc2::InstantCommand([this] { m_drive.ZeroHeading(); }).ToPtr());

  // Maps to 6 on keyboard
  m_operatorController.Button(9).OnTrue(
      m_rightClimb.MoveToPositionAbsolute(20_in).AlongWith(
          m_leftClimb.MoveToPositionAbsolute(20_in)));

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
      (frc2::InstantCommand([this] { m_autoAcquiringNote = true; })
           .ToPtr()
           .AndThen(IntakeTarget())
           .AndThen(frc2::InstantCommand([this] {
                      m_autoAcquiringNote = false;
                    }).ToPtr())
           .AndThen(frc2::InstantCommand([this] {
                      if (!m_state.m_active) {
                        m_state.m_currentState = RobotState::ScoringSubwoofer;
                        m_state.SetDesiredState();
                      }
                    }).ToPtr()))
          .WithTimeout(20_s)
          .FinallyDo([this] { m_autoAcquiringNote = false; }));
}
#endif

frc2::Command* RobotContainer::GetAutonomousCommand() {
  auto autoType = m_autoChooser.GetSelectedValue();
  autoCommand = m_autoFactory.GetAuto(autoType);

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

units::degree_t RobotContainer::CurveRotation(double sensitivity, double val,
                                              double inMin, double inMax,
                                              double outMin, double outMax) {
  auto normalizedValue =
      TurnToPose::NormalizeScalar(val, inMin, inMax, -1.0, 1.0);
  double cubedVal = std::clamp(pow(normalizedValue, 3), -1.0, 1.0);
  double rampedVal = std::clamp(
      (sensitivity * (cubedVal) + (1.0 - sensitivity) * val), -1.0, 1.0);
  double summedVal = cubedVal + rampedVal;
  return units::degree_t(
      TurnToPose::NormalizeScalar(summedVal, -1.0, 1.0, outMin, outMax));
}

void RobotContainer::Periodic() {
  frc::SmartDashboard::PutBoolean("Disable Climb Limits", m_ignoreClimbLimits);

  frc::SmartDashboard::PutData("Robot2d", &m_mech);

  frc::SmartDashboard::PutBoolean("TURN TO POSE AT GOAL",
                                  m_turnToPose.AtGoal());

  m_turnToPose.Update();
  auto targets = m_tracker.GetTargets();
  m_tracker.UpdateTrackedTargets(targets);

  if (m_intake.NotePresent()) {
    // Note is present, get ready to score it
    auto locations = DrivingCommands::GetSortedLocations(m_drive.GetPose());
    bool inRange = false;

    for (auto& location : locations) {
      if (location.hypotDistance <= location.locationRadius) {
        inRange = m_aimbotEnabled && true;
        m_turnToPose.SetTargetPose(location.trackedPose);

        if (inRange && m_autoScoringEnabled && location.scoringRadius &&
            location.scoringDirection &&
            location.hypotDistance <= location.scoringRadius.value() &&
            !autoScoreCommand.IsScheduled() && m_ramping) {
          autoScoreCommand =
              (m_leds.AutoScoring()
                   .AndThen(ScoringCommands::Score(
                       [location] { return location.scoringDirection.value(); },
                       &m_scoring, &m_intake, &m_arm))
                   .AndThen(m_leds.Idling()))
                  .WithTimeout(4_s)
                  .FinallyDo([this] { m_ramping = false; });
          // ? How do we know that the shooter is ramped up enough?
          // Re: https://i.ytimg.com/vi/8Jul5SuPYJc/mqdefault.jpg
          autoScoreCommand.Schedule();
        } else if (inRange && m_autoScoringEnabled &&
                   location.scoringDirection &&
                   !autoScoreCommand.IsScheduled() && !m_ramping) {
          // ScoringDirection direction = location.scoringDirection
          //                                  ?
          //                                  location.scoringDirection.value()
          //                                  : ScoringDirection::AmpSide;
          m_leds.RampingAsync();
          // m_scoring.StartScoringRamp(direction);
          m_ramping = true;
        }

        frc::SmartDashboard::PutNumber(
            "TURN TO POSE TARGET deg",
            location.trackedPose.Rotation().Degrees().value());

        break;
      }
    }

    m_shouldAim = m_aimbotEnabled && inRange;

    if (!m_shouldAim || !m_autoScoringEnabled) {
      m_ramping = false;
    }
  } else {
    auto bestTarget = m_tracker.GetBestTarget(targets);
    auto targetPose = m_tracker.GetBestTargetPose(targets);

    if (targetPose) {
      m_drive.GetField()->GetObject("note_target")->SetPose(targetPose.value());
    }

    frc::SmartDashboard::PutBoolean("HAS TARGET LOCK",
                                    m_tracker.HasTargetLock(targets));

    if (bestTarget) {
      m_shouldAim = m_aimbotEnabled;
      auto centerDiff = bestTarget.value().centerX;

      auto normalizedTargetAngle = TurnToPose::NormalizeScalar(
          centerDiff.value(), VisionConstants::kMinAngleDeg,
          VisionConstants::kMaxAngleDeg, -1.0, 1.0);
      normalizedTargetAngle =
          std::clamp(pow(normalizedTargetAngle, 3), -1.0, 1.0);
      centerDiff = units::degree_t(TurnToPose::NormalizeScalar(
          normalizedTargetAngle, -1.0, 1.0, VisionConstants::kMinAngleDeg,
          VisionConstants::kMaxAngleDeg));

      // centerDiff = CurveRotation(0.375, centerDiff.value(), -30.0, 30.0,
      // -30.0, 30.0);

      m_turnToPose.SetTargetAngleRelative(-centerDiff);

      frc::SmartDashboard::PutNumber("TURN TO ANGLE relative target deg",
                                     -centerDiff.value());
    } else {
      m_shouldAim = false;
    }
  }

  m_ignoreClimbLimits = m_ignoreLimitChooser.GetSelected();
}

std::optional<frc::Rotation2d> RobotContainer::GetRotationTargetOverride() {
  auto targetPose = m_turnToPose.GetTargetPose();

  if (m_autoAcquiringNote && targetPose) {
    auto angle = m_turnToPose.GetTargetHeading();
    // angle = m_drive.GetPose().Rotation().Degrees();
    frc::SmartDashboard::PutNumber("Overridden PP angle", angle.value());
    return angle;
  }

  return std::nullopt;
}

void RobotContainer::ToggleAimbot() {
  m_aimbotEnabled = !m_aimbotEnabled;
  frc::SmartDashboard::PutBoolean("aimbot enabled", m_aimbotEnabled);
}

void RobotContainer::ToggleAutoScoring() {
  m_autoScoringEnabled = !m_autoScoringEnabled;
  frc::SmartDashboard::PutBoolean("auto-scoring enabled", m_autoScoringEnabled);
}

frc2::CommandPtr RobotContainer::MoveToIntakePose() {
  return frc2::DeferredCommand(
             [this] {
               auto targets = m_tracker.GetTargets();
               auto targetPose = m_tracker.GetBestTargetPose(targets);

               targetPose = frc::Pose2d(targetPose.value().Translation(),
                                        m_turnToPose.GetTargetHeading());

               return pathplanner::AutoBuilder::pathfindToPose(
                   targetPose.value(), AutoConstants::kMovementConstraints,
                   0.0_mps,  // Goal end velocity in meters/sec
                   0.0_m  // Rotation delay distance in meters. This is how far
                          // the robot should travel before attempting to
                          // rotate.
               );
             },
             {})
      .ToPtr();
}

frc2::CommandPtr RobotContainer::IntakeTarget() {
  return frc2::DeferredCommand(
             [this] {
               return (MoveToIntakePose().AlongWith(
                           IntakingCommands::Intake(&m_intake, &m_scoring)))
                   .WithTimeout(10_s)
                   .Until([this] {
                     return StateSubsystem::IsControllerActive(
                         m_driverController, m_operatorController);
                   });
             },
             {})
      .ToPtr();
}