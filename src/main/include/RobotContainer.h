// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <AHRS.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "subsystems/LeftClimbSubsystem.h"
#include "subsystems/RightClimbSubsystem.h"
#include "subsystems/ScoringSubsystem.h"
#include "subsystems/StateSubsystem.h"
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

 private:
  // The driver's controller
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};
  // frc::XboxController
  // m_operatorController{OIConstants::kOperatorControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive{&m_vision};

  LedSubsystem m_leds;

  frc2::CommandPtr m_defaultAuto =
      pathplanner::PathPlannerAuto(AutoConstants::kDefaultAutoName).ToPtr();
  frc2::CommandPtr m_3inAmp = pathplanner::PathPlannerAuto("3 in Amp").ToPtr();
  frc2::CommandPtr m_2noteAuto =
      pathplanner::PathPlannerAuto("2 Note Auto").ToPtr();
  frc2::CommandPtr m_4noteAuto =
      pathplanner::PathPlannerAuto("4 Note Auto").ToPtr();
  frc2::CommandPtr m_kepler =
      pathplanner::PathPlannerAuto("Kepler Auto").ToPtr();
  frc2::CommandPtr m_kepler2 =
      pathplanner::PathPlannerAuto("Kepler 2 Auto").ToPtr();

    std::unordered_map<std::string, frc2::Command*> autoCommands = {
        {AutoConstants::kDefaultAutoName, m_defaultAuto.get()},
        {"3 in Amp", m_3inAmp.get()},
        {"2 Note Auto", m_2noteAuto.get()},
        {"4 Note Auto", m_4noteAuto.get()},
        {"Kepler Auto", m_kepler.get()},
        {"Kepler 2 Auto", m_kepler2.get()},
    };

  // The chooser for the autonomous routines
  frc::SendableChooser<std::string> m_chooser;

#ifdef TEST_SWERVE_BOT

#endif

#ifndef TEST_SWERVE_BOT
  LeftClimbSubsystem m_leftClimb;
  RightClimbSubsystem m_rightClimb;
  IntakeSubsystem m_intake;
  ScoringSubsystem m_scoring;

  Subsystems_t m_subsystems = {.drive = &m_drive,
                               .leftClimb = &m_leftClimb,
                               .rightClimb = &m_rightClimb,
                               .intake = &m_intake,
                               .scoring = &m_scoring,
                               .led = &m_leds};

  frc2::CommandXboxController m_operatorController{
      OIConstants::kOperatorControllerPort};

  StateSubsystem m_state{m_subsystems, m_driverController,
                         m_operatorController};

  void ConfigureAutoBindings();
#endif

  Vision m_vision;

  AHRS gyro2{frc::SPI::Port::kMXP};

  void ConfigureButtonBindings();
};
