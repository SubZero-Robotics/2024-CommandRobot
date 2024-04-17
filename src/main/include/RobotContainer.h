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
#include "utils/TargetTracker.h"
#include "utils/Vision.h"

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

 private:
  frc::Mechanism2d m_mech{1, 1};

  // The driver's controller
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive{&m_vision};

  LedSubsystem m_leds;

  // The chooser for the autonomous routines
  frc::SendableChooser<AutoConstants::AutoType> m_chooser;

#ifdef TEST_SWERVE_BOT

#endif

#ifndef TEST_SWERVE_BOT
  LeftClimbSubsystem m_leftClimb{(frc::MechanismObject2d*)m_mech.GetRoot(
      "Climber Left", MechanismConstants::kClimberLeftX,
      MechanismConstants::kClimberLeftY)};
  RightClimbSubsystem m_rightClimb{(frc::MechanismObject2d*)m_mech.GetRoot(
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

  TargetTracker tracker{-20_deg, 15_in, 0.33,
                        &m_intake, &m_scoring, &m_drive};
  TurnToPose m_turnToPose{[this] { return m_drive.GetPose(); },
                          [this] { return m_drive.GetField(); }};

  void ConfigureAutoBindings();
#endif

  Vision m_vision;

  void RegisterAutos();

  void ConfigureButtonBindings();

  frc2::CommandPtr autoCommand = frc2::InstantCommand([] {}).ToPtr();

  bool m_aimbotEnabled = false;
  bool m_shouldAim = false;
  void ToggleAimbot();
};
