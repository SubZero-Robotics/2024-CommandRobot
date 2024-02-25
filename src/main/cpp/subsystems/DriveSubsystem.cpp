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
}

void DriveSubsystem::SimulationPeriodic() {
  logDrivebase();
  frc::ChassisSpeeds chassisSpeeds = kDriveKinematics.ToChassisSpeeds(
      m_frontLeft.GetState(), m_rearLeft.GetState(), m_frontRight.GetState(),
      m_rearRight.GetState());
  m_gyroSimAngle.Set(m_gyro.GetAngle() +
                     (chassisSpeeds.omega.convert<units::deg_per_s>().value() *
                      DriveConstants::kLoopTime.value()));
  poseEstimator.Update(-m_gyro.GetRotation2d(), GetModulePositions());

  m_field.SetRobotPose(poseEstimator.GetEstimatedPosition());
};

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  if (frc::RobotBase::IsReal()) {
    poseEstimator.UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                                 m_gyro.GetRotation2d(), GetModulePositions());
    logDrivebase();

    auto visionPoses = m_vision->UpdateEstimatedGlobalPose(poseEstimator);

    auto updatedPose = poseEstimator.GetEstimatedPosition();
    // Might need to get the pose from somewhere else?
    frc::Pose2d finalPose = m_lastGoodPosition;
    // ? Do we need to flip this first based on alliance?
    // https://github.com/Hemlock5712/2023-Robot/blob/dd5ac64587a3839492cfdb0a28d21677d465584a/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java#L149

    if (updatedPose.X() > 0_m && updatedPose.X() <= 100_m &&
        updatedPose.Y() > 0_m && updatedPose.Y() <= 100_m) {
      finalPose = updatedPose;
    } else {
      auto cam1Est = visionPoses.first;
      auto cam2Est = visionPoses.second;

      if (cam1Est.has_value()) {
        finalPose = cam1Est.value().estimatedPose.ToPose2d();
      } else if (cam2Est.has_value()) {
        finalPose = cam2Est.value().estimatedPose.ToPose2d();
      }
    }

    ConsoleLogger::getInstance().logVerbose(
        "DriveSubsystem PoseEstimator final pose", finalPose);
    m_lastGoodPosition = finalPose;
    m_field.SetRobotPose(finalPose);
  };
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           bool rateLimit, units::second_t periodSeconds) {
  double xSpeedCommanded;
  double ySpeedCommanded;

  ShuffleboardLogger::getInstance().logInfo("xSpeed", xSpeed.value());
  ShuffleboardLogger::getInstance().logInfo("ySpeed", ySpeed.value());
  ShuffleboardLogger::getInstance().logInfo("Rotation", rot.value());

  double currentTime = wpi::Now() * 1e-6;
  double elapsedTime = currentTime - m_prevTime;
  periodSeconds = units::second_t(elapsedTime);

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
    double inputTranslationMag =
        sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

    // Calculate the direction slew rate based on an estimate of the lateral
    // acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate =
          abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0;  // some high number that means the slew rate
                                  // is effectively instantaneous
    }

    double currentTime = wpi::Now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils::AngleDifference(inputTranslationDir,
                                                   m_currentTranslationDir);

    driveLoopTime = units::second_t{elapsedTime};

    if (angleDif < 0.45 * std::numbers::pi) {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
    } else if (angleDif > 0.85 * std::numbers::pi) {
      if (m_currentTranslationMag >
          1e-4) {  // some small number to avoid floating-point errors with
                   // equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      } else {
        m_currentTranslationDir =
            SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
    } else {
      m_currentTranslationDir = SwerveUtils::StepTowardsCircular(
          m_currentTranslationDir, inputTranslationDir,
          directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.Calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.Calculate(rot.value());

  } else {
    xSpeedCommanded = xSpeed.value();
    ySpeedCommanded = ySpeed.value();
    m_currentRotation = rot.value();
  }

  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeedCommanded * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeedCommanded * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      m_currentRotation * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::Discretize(
                frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    frc::Rotation2d(units::degree_t{m_gyro.GetAngle()})),
                DriveConstants::kLoopTime)
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

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
      kDriveKinematics.ToSwerveModuleStates(speeds));
}

frc::ChassisSpeeds DriveSubsystem::getSpeed() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontRight.GetState();
  auto rl = m_rearLeft.GetState();
  auto rr = m_rearRight.GetState();

  return kDriveKinematics.ToChassisSpeeds(fl, fr, rl, rr);
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
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
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
  return {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
          m_frontRight.GetPosition(), m_rearRight.GetPosition()};
}

units::degree_t DriveSubsystem::GetHeading() {
  return frc::Rotation2d(units::degree_t{m_gyro.GetAngle()}).Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

double DriveSubsystem::GetTurnRate() { return m_gyro.GetRate(); }

frc::Pose2d DriveSubsystem::GetPose() {
  return poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_lastGoodPosition = pose;
  poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), pose);
  // ResetEncoders();
}
