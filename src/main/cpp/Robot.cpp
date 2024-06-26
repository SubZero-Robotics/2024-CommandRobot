// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <wpinet/uv/Error.h>

#include "Constants.h"
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "wpi/DataLog.h"

void Robot::RobotInit() {
  // if (RobotBase::IsReal()) {
  //   if (frc::RobotController::GetSerialNumber() !=
  //       RobotConstants::kRoborioSerialNumber) {
  //     std::cout << "Error: wrong robot\n";
  //     throw wpi::uv::Error(108);
  //   }
  // }
  frc::DataLogManager::Start();

  // Logs joystick data without hogging NT bandwidth
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  m_container.Initialize();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.ClearCurrentStateCommand();
  m_container.DisableSubsystems();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

  m_container.StartIdling();
  m_container.StopMotors();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() { m_container.Periodic(); }

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
