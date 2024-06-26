// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <subzero/autonomous/AutoFactory.h>
#include <subzero/frc/smartdashboard/TaggedChooser.h>
#include <subzero/target/TurnToPose.h>
#include <subzero/vision/PhotonVisionEstimators.h>
#include <subzero/vision/TargetTracker.h>

#include <vector>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "subsystems/LeftClimbSubsystem.h"
#include "subsystems/RightClimbSubsystem.h"
#include "subsystems/ScoringSubsystem.h"
#include "subsystems/StateSubsystem.h"
#include "utils/Commands/DriveCommands.h"
#include "utils/Commands/FunniCommands.h"
#include "utils/Commands/IntakeCommands.h"
#include "utils/Commands/ScoreCommands.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void ClearCurrentStateCommand();
  void StartIdling();
  void ResetPose();
  void DisableSubsystems();
  void Initialize();
  void StopMotors();
  void Periodic();
  units::degree_t CurveRotation(double sensitivity, double val, double inMin,
                                double inMax, double outMin, double outMax);

 private:
  frc::Mechanism2d m_mech{0.5, 0.5};

  // The driver's controller
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive{&m_vision};

  LedSubsystem m_leds;

  bool m_ignoreClimbLimits = false;

  // The chooser for the autonomous routines
  subzero::TaggedChooser<AutoConstants::AutoType> m_autoChooser{
      AutoConstants::kChooserEntries, AutoConstants::kChooserGroups,
      "Auto Selector"};

  subzero::AutoFactory<AutoConstants::AutoType> m_autoFactory{
      AutoConstants::kPpAutos};

  frc::SendableChooser<bool> m_ignoreLimitChooser;

#ifdef TEST_SWERVE_BOT

#endif

#ifndef TEST_SWERVE_BOT
  LeftClimbSubsystem m_leftClimb{
      [this] { return m_ignoreClimbLimits; },
      (frc::MechanismObject2d*)m_mech.GetRoot(
          "Climber Left", MechanismConstants::kClimberLeftX,
          MechanismConstants::kClimberLeftY)};
  RightClimbSubsystem m_rightClimb{
      [this] { return m_ignoreClimbLimits; },
      (frc::MechanismObject2d*)m_mech.GetRoot(
          "Climber Right", MechanismConstants::kClimberRightX,
          MechanismConstants::kClimberRightY)};
  frc::MechanismRoot2d* armRoot = m_mech.GetRoot(
      "Arm Root", MechanismConstants::kArmRootX, MechanismConstants::kArmRootY);
  frc::MechanismLigament2d* armPost = armRoot->Append<frc::MechanismLigament2d>(
      "Arm Post", MechanismConstants::kArmPostX,
      MechanismConstants::kArmPostAngle);
  ArmSubsystem m_arm{(frc::MechanismObject2d*)armPost};
  IntakeSubsystem m_intake;
  ScoringSubsystem m_scoring;

  Subsystems_t m_subsystems = {.drive = &m_drive,
                               .leftClimb = &m_leftClimb,
                               .rightClimb = &m_rightClimb,
                               .intake = &m_intake,
                               .scoring = &m_scoring,
                               .led = &m_leds,
                               .arm = &m_arm};

  frc2::CommandXboxController m_operatorController{
      OIConstants::kOperatorControllerPort};

  StateSubsystem m_state{m_subsystems, m_driverController,
                         m_operatorController};

  subzero::TargetTracker m_tracker{
      {// Camera angle
       VisionConstants::kCameraAngle,
       // Camera lens height
       VisionConstants::kCameraLensHeight,
       // Confidence threshold
       VisionConstants::kConfidenceThreshold,
       // Limelight name
       VisionConstants::kLimelightName,
       // Gamepiece width
       VisionConstants::kNoteWidth,
       // Focal length
       VisionConstants::focalLength,
       // Sim gamepiece pose
       VisionConstants::kSimGamepiecePose,
       // Gamepiece rotation
       VisionConstants::kGamepieceRotation,
       // Trig-based distance percentage
       VisionConstants::kTrigDistancePercentage,
       // Area percentage threshold
       VisionConstants::kAreaPercentageThreshold,
       // Max # of tracked objects
       VisionConstants::kMaxTrackedTargets,
       // Default pose to use when tracked target isn't found
       frc::Pose2d{100_m, 100_m, frc::Rotation2d{0_deg}}},
      [this] { return m_drive.GetPose(); },
      [this] { return m_drive.GetField(); }};

  subzero::TurnToPose m_turnToPose{
      {// Rotation constraints
       frc::TrapezoidProfile<units::radians>::Constraints{
           TurnToPoseConstants::kProfileVelocity,
           TurnToPoseConstants::kProfileAcceleration},
       // Turn P
       TurnToPoseConstants::kTurnP,
       // Turn I
       TurnToPoseConstants::kTurnI,
       // Turn D
       TurnToPoseConstants::kTurnD,
       // Translation P
       TurnToPoseConstants::kTurnTranslationP,
       // Translation I
       TurnToPoseConstants::kTurnTranslationI,
       // Translation D
       TurnToPoseConstants::kTurnTranslationD,
       // Pose tolerance
       TurnToPoseConstants::kPoseTolerance},
      [this] { return m_drive.GetPose(); },
      [this] { return m_drive.GetField(); }};

  photon::PhotonPoseEstimator poseFront{
      // layout
      VisionConstants::kTagLayout,
      // strategy
      VisionConstants::kPoseStrategy,
      // camera name
      photon::PhotonCamera{VisionConstants::kFrontCamera},
      // offsets
      VisionConstants::kRobotToCam};

  photon::PhotonPoseEstimator poseRear{
      // layout
      VisionConstants::kTagLayout,
      // strategy
      VisionConstants::kPoseStrategy,
      // camera name
      photon::PhotonCamera{VisionConstants::kRearCamera},
      // offsets
      VisionConstants::kRobotToCam2};

  std::vector<subzero::PhotonVisionEstimators::PhotonCameraEstimator>
      poseCameras{
          subzero::PhotonVisionEstimators::PhotonCameraEstimator(poseFront),
          subzero::PhotonVisionEstimators::PhotonCameraEstimator(poseRear),
      };

  void ConfigureAutoBindings();
#endif

  subzero::PhotonVisionEstimators m_vision{poseCameras,
                                           VisionConstants::kSingleTagStdDevs,
                                           VisionConstants::kMultiTagStdDevs};

  void RegisterAutos();
  void ConfigureButtonBindings();
  frc2::CommandPtr MoveToIntakePose();
  frc2::CommandPtr IntakeTarget();

  frc2::CommandPtr autoCommand = frc2::InstantCommand([] {}).ToPtr();
  frc2::CommandPtr autoScoreCommand = frc2::InstantCommand([] {}).ToPtr();

  bool m_aimbotEnabled = false;
  bool m_autoScoringEnabled = false;
  bool m_shouldAim = false;
  bool m_autoAcquiringNote = false;
  bool m_ramping = false;
  void ToggleAimbot();
  void ToggleAutoScoring();
  std::optional<frc::Rotation2d> GetRotationTargetOverride();
};
