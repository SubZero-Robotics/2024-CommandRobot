// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADXRS450_Gyro.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>
#include <networktables/StructArrayTopic.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <string>

#include "Constants.h"
#include "MAXSwerveModule.h"
#include "utils/ConsoleLogger.h"
#include "utils/ITurnToTarget.h"
#include "utils/Vision.h"

// For sim to work
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  explicit DriveSubsystem(Vision* vision);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   *
   * @param periodSeconds Time between periodic loops
   *
   * @param turnToTarget Will apply a correction based on the PID's desired
   * state
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, bool rateLimit, units::second_t periodSeconds,
             ITurnToTarget* turnToTarget = nullptr);

  /**
   * Drives the robot based on a ChassisSpeeds
   *
   * @param speeds        ChassisSpeeds to drive based on
   */
  void Drive(frc::ChassisSpeeds speeds);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Publishes the swerve drive states to NT in an advantagescope friendly
   * format
   */
  void logDrivebase();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the rotation of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  frc::Rotation2d GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  frc::Field2d* GetField() { return &m_field; }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  void SetRotationOffset(units::degree_t offset);

  frc::ChassisSpeeds getSpeed();

  ctre::phoenix6::hardware::Pigeon2* getGyro() { return &m_gyro1; }

  wpi::array<frc::SwerveModulePosition, 4U> GetModulePositions() const;

  static void LogSpeeds(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp);
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);

  frc::SwerveDriveKinematics<4> m_driveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  void logMotorState(MAXSwerveModule& motor, std::string key);

  frc::ChassisSpeeds GetSpeedsFromJoystick(units::meters_per_second_t xSpeed,
                                           units::meters_per_second_t ySpeed,
                                           units::radians_per_second_t rot,
                                           bool fieldRelative);

  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  uint8_t logCounter = 0;

  ctre::phoenix6::hardware::Pigeon2 m_gyro1{CANConstants::kPigeonCanId1, "rio"};

  ctre::phoenix6::sim::Pigeon2SimState& m_gyro1Sim = m_gyro1.GetSimState();

  // time last loop took, "deltatime"
  units::second_t driveLoopTime = DriveConstants::kLoopTime;

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter{
      DriveConstants::kMagnitudeSlewRate / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{
      DriveConstants::kRotationalSlewRate / 1_s};
  double m_prevTime = wpi::Now() * 1e-6;

  // Odometry class for tracking robot pose
  // 4 defines the number of modules

  frc::SwerveDrivePoseEstimator<4> poseEstimator{
      m_driveKinematics,
      GetHeading(),
      {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
       m_frontRight.GetPosition(), m_rearRight.GetPosition()},
      frc::Pose2d{0_m, 0_m, 0_rad},
      VisionConstants::kDrivetrainStd,
      VisionConstants::kVisionStd};
  nt::StructArrayPublisher<frc::SwerveModuleState> m_publisher;

  // Pose viewing
  frc::Field2d m_field;
  frc::Pose2d m_lastGoodPosition;

  Vision* m_vision;

  std::unique_ptr<frc2::sysid::SysIdRoutine> m_sysIdRoutine;
};
