// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

#include "Constants.h"

class MAXSwerveModule {
 public:
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  MAXSwerveModule(int driveCANId, int turningCANId, double chassisAngularOffset,
                  frc::SimpleMotorFeedforward<units::meters>&);

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  frc::SwerveModuleState GetState() const;

  frc::SwerveModuleState GetSimState() const;

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition() const;

  frc::SwerveModulePosition GetSimPosition() const;

  frc::Rotation2d GetRotation() const;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  frc::SwerveModuleState GetDesiredState() { return m_desiredState; }

  void simUpdateDrivePosition(const frc::SwerveModuleState& state);

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

 private:
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkRelativeEncoder m_drivingEncoder =
      m_drivingSparkMax.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                                   CANConstants::kTicksPerMotorRotation);
  rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder =
      m_turningSparkMax.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  rev::SparkPIDController m_drivingPIDController =
      m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController =
      m_turningSparkMax.GetPIDController();

  double m_chassisAngularOffset = 0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};

  frc::SimpleMotorFeedforward<units::meters>& m_feedForward;

  // Values for simulation
  units::meter_t m_simDriveEncoderPosition;
  units::meters_per_second_t m_simDriveEncoderVelocity;

  units::radian_t m_simCurrentAngle;
};
