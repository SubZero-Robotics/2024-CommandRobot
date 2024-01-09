// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/IterativeRobotBase.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "utils/ShuffleboardLogger.h"
#include "commands/Funni.h"
#include "commands/LEDToggleCommand.h"
#include "commands/RotateWristCommand.h"
#include "commands/IntakeInCommand.h"
#include "commands/IntakeOutCommand.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  m_wrist = std::make_unique<WristSubsystem>();

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

    m_chooser.SetDefaultOption("Leave Community", pathplanner::PathPlannerAuto("Leave COM").ToPtr().Unwrap().get());
    shuffleboardLogger.logVerbose("Auto Modes", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {
  frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
      .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));

    m_wrist->SetDefaultCommand(
        RotateWrist(m_wrist.get(), [this] {
            return -m_operatorController.GetRightY();
        }));
    
    frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kRightBumper).WhileTrue(IntakeIn(&m_intake).ToPtr());

    frc2::JoystickButton(&m_operatorController,
                       frc::XboxController::Button::kLeftBumper).WhileTrue(IntakeOut(&m_intake).ToPtr());

    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kX).OnTrue(LEDToggle(&m_leds).ToPtr());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}
