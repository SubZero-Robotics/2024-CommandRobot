// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>

class MAXSwerveModule {
 public:
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  MAXSwerveModule(int driveCANId, int turningCANId,
                  double chassisAngularOffset);

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  frc::SwerveModuleState GetState() const;

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  void SetDesiredState(const frc::SwerveModuleState& state);

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

 private:
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkMaxRelativeEncoder m_drivingEncoder =
      m_drivingSparkMax.GetEncoder();
  rev::SparkMaxAbsoluteEncoder m_turningAbsoluteEncoder =
      m_turningSparkMax.GetAbsoluteEncoder(
          rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

  rev::SparkMaxPIDController m_drivingPIDController =
      m_drivingSparkMax.GetPIDController();
  rev::SparkMaxPIDController m_turningPIDController =
      m_turningSparkMax.GetPIDController();

  double m_chassisAngularOffset = 0;
  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0},
                                        frc::Rotation2d()};
};
