// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.h"
#include "frc/DataLogManager.h"
#include "utils/ShuffleboardLogger.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem(Vision* vision)
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_vision{vision} {
  // put a field2d in NT
  frc::SmartDashboard::PutData("Field", &m_field);

  pathplanner::AutoBuilder::configureHolonomic(
      [this]() { return GetPose(); },
      [this](frc::Pose2d pose) -> void { ResetOdometry(pose); },
      [this]() -> frc::ChassisSpeeds { return getSpeed(); },
      [this](frc::ChassisSpeeds speeds) -> void { Drive(speeds); },
      AutoConstants::PathConfig,
      []() {
        // Boolean supplier that controls when the path will be mirrored for the
        // red alliance This will flip the path being followed to the red side
        // of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this);

  m_publisher =
      nt::NetworkTableInstance::GetDefault()
          .GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates")
          .Publish();

  m_driveModules = {&m_frontLeft, &m_rearLeft, &m_frontRight, &m_rearRight};

  m_sysIdRoutine =
      makeSysIdRoutine(kMotorNames, m_driveModules, MotorType::DriveMotor);
}

std::unique_ptr<frc2::sysid::SysIdRoutine> DriveSubsystem::makeSysIdRoutine(
    std::vector<std::string> motorNames, std::vector<MAXSwerveModule*> modules,
    MotorType motorType) {
  return std::make_unique<frc2::sysid::SysIdRoutine>(
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this, motorType](units::volt_t driveVoltage) {
            for (auto* module : m_driveModules) {
              module->SetMotorVoltage(motorType, driveVoltage);
            }
          },
          [this, motorNames, motorType](frc::sysid::SysIdRoutineLog* log) {
            ConsoleWriter.logVerbose("LOG WRITE", "log write called%s", "");
            for (size_t i = 0; i < motorNames.size(); i++) {
              // log->Motor(motorNames[i])
              //     .voltage(m_driveModules[i]->Get(motorType) *
              //              frc::RobotController::GetBatteryVoltage())
              //     .position(
              //         units::meter_t{m_driveModules[i]->GetDistance(motorType)})
              //     .velocity(units::meters_per_second_t{
              //         m_driveModules[i]->GetRate(motorType)});
              log->Motor(motorNames[i])
                  .voltage(
                      units::volt_t{m_driveModules[i]->Get(motorType) *
                                    frc::RobotController::GetBatteryVoltage()})
                  .position(
                      units::meter_t{m_driveModules[i]->GetDistance(motorType)})
                  .velocity(units::meters_per_second_t{
                      m_driveModules[i]->GetRate(motorType)});
            }
          },
          this});
}

void DriveSubsystem::SimulationPeriodic() {
  logDrivebase();

  frc::ChassisSpeeds chassisSpeeds = m_driveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(),
      m_rearRight.GetState());

  m_gyro1Sim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_gyro1Sim.SetRawYaw(
      GetHeading().Degrees() +
      units::degree_t(chassisSpeeds.omega.convert<units::deg_per_s>().value() *
                      DriveConstants::kLoopTime.value()));

  poseEstimator.Update(GetHeading().Degrees(), GetModulePositions());
  m_field.SetRobotPose(poseEstimator.GetEstimatedPosition());
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  if (frc::RobotBase::IsReal()) {
    poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(), GetHeading(),
                                 GetModulePositions());
    logDrivebase();

    m_vision->UpdateEstimatedGlobalPose(poseEstimator, true);

    auto updatedPose = poseEstimator.GetEstimatedPosition();
    // https://github.com/Hemlock5712/2023-Robot/blob/dd5ac64587a3839492cfdb0a28d21677d465584a/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L149
    m_lastGoodPosition = updatedPose;
    m_field.SetRobotPose(updatedPose);
  }
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           bool rateLimit, units::second_t periodSeconds,
                           ITurnToTarget* turnToTarget) {
  auto now = frc::Timer::GetFPGATimestamp();
  auto dif = now - driveLoopTime;

  if (dif > 30_ms) {
    ConsoleWriter.logVerbose("EVAN", "AHHHH BAD NOOO CRYYYY TERRIBLE %s", "");
  }

  auto joystickSpeeds =
      GetSpeedsFromJoystick(xSpeed, ySpeed, rot, fieldRelative);

  if (turnToTarget) {
    if (joystickSpeeds.omega.value() == 0) {
      joystickSpeeds = turnToTarget->BlendWithInput(joystickSpeeds, 1);
    } else {
      joystickSpeeds = turnToTarget->BlendWithInput(
          joystickSpeeds, TurnToPoseConstants::kBlendRatio);
    }
  }

  auto states = m_driveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds::Discretize(joystickSpeeds, dif));

  driveLoopTime = now;

  m_driveKinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

  logMotorState(m_frontLeft, std::string("Front Left Motor"));
  logMotorState(m_frontRight, std::string("Front Right Motor"));
  logMotorState(m_rearLeft, std::string("Rear Left Motor"));
  logMotorState(m_rearRight, std::string("Rear Right Motor"));
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds) {
  DriveSubsystem::SetModuleStates(
      m_driveKinematics.ToSwerveModuleStates(speeds));
}

frc::ChassisSpeeds DriveSubsystem::GetSpeedsFromJoystick(
    units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
    units::radians_per_second_t rot, bool fieldRelative) {
  return fieldRelative
             ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                   xSpeed * DriveConstants::kMaxSpeed.value(),
                   ySpeed * DriveConstants::kMaxSpeed.value(),
                   rot * DriveConstants::kMaxAngularSpeed.value(), GetHeading())
             : frc::ChassisSpeeds{xSpeed, ySpeed, rot};
}

frc::ChassisSpeeds DriveSubsystem::getSpeed() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontRight.GetState();
  auto rl = m_rearLeft.GetState();
  auto rr = m_rearRight.GetState();

  return m_driveKinematics.ToChassisSpeeds(fl, fr, rl, rr);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::logMotorState(MAXSwerveModule& motor, std::string key) {
  ShuffleboardLogger::getInstance().logInfo(key,
                                            motor.GetState().speed.value());
}

void DriveSubsystem::logDrivebase() {
  std::vector states_vec = {m_frontLeft.GetState(), m_frontRight.GetState(),
                            m_rearLeft.GetState(), m_rearRight.GetState()};
  std::span<frc::SwerveModuleState, 4> states(states_vec.begin(),
                                              states_vec.end());

  m_publisher.Set(states);
}

void DriveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                          units::second_t timestamp) {
  poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp);
}

void DriveSubsystem::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                          units::second_t timestamp,
                                          const Eigen::Vector3d& stdDevs) {
  wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
  poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp, newStdDevs);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  m_driveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                          DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  // Might need to swap these?
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

wpi::array<frc::SwerveModulePosition, 4U> DriveSubsystem::GetModulePositions()
    const {
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
          m_rearLeft.GetPosition(), m_rearRight.GetPosition()};
}

frc::Rotation2d DriveSubsystem::GetHeading() { return m_gyro1.GetRotation2d(); }

void DriveSubsystem::ZeroHeading() { m_gyro1.Reset(); }

// TODO: Return the rate of the currently used gyro
double DriveSubsystem::GetTurnRate() { return m_gyro1.GetRate(); }

frc::Pose2d DriveSubsystem::GetPose() {
  return poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_lastGoodPosition = pose;
  poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), pose);
}

frc2::CommandPtr DriveSubsystem::SysIdQuasistatic(
    frc2::sysid::Direction direction) {
  return m_sysIdRoutine->Quasistatic(direction);
}
frc2::CommandPtr DriveSubsystem::SysIdDynamic(
    frc2::sysid::Direction direction) {
  return m_sysIdRoutine->Dynamic(direction);
}