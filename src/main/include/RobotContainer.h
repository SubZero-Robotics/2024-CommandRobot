// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LeftClimbSubsystem.h"
#include "subsystems/RightClimbSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "subsystems/ScoringSubsystem.h"
#include "subsystems/StateSubsystem.h"

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

 private:
  // The driver's controller
  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};
  // frc::XboxController m_operatorController{OIConstants::kOperatorControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  LedSubsystem m_leds;

  frc2::CommandPtr m_defaultAuto = pathplanner::PathPlannerAuto(AutoConstants::kDefaultAutoName).ToPtr();

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

#ifdef TEST_SWERVE_BOT

#endif

#ifndef TEST_SWERVE_BOT
  LeftClimbSubsystem m_leftClimb;
  RightClimbSubsystem m_rightClimb;
  IntakeSubsystem m_intake;
  ScoringSubsystem m_scoring;

  Subsystems_t m_subsystems = {
    .drive = &m_drive,
    .leftClimb = &m_leftClimb,
    .rightClimb = &m_rightClimb,
    .intake = &m_intake,
    .scoring = &m_scoring,
    .led = &m_leds
  };

  StateSubsystem m_state{m_subsystems, m_driverController};
#endif

  void ConfigureButtonBindings();
};
