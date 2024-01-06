// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/MAXSwerveModule.h"

#include <frc/geometry/Rotation2d.h>

#include <numbers>

#include "Constants.h"

using namespace ModuleConstants;

MAXSwerveModule::MAXSwerveModule(const int drivingCANId, const int turningCANId,
                                 const double chassisAngularOffset)
    : m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless) {
  // Factory reset, so we get the SPARKS MAX to a known state before configuring
  // them. This is useful in case a SPARK MAX is swapped out.
  m_drivingSparkMax.RestoreFactoryDefaults();
  m_turningSparkMax.RestoreFactoryDefaults();

  // Apply position and velocity conversion factors for the driving encoder. The
  // native units for position and velocity are rotations and RPM, respectively,
  // but we want meters and meters per second to use with WPILib's swerve APIs.
  m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
  m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

  // Apply position and velocity conversion factors for the turning encoder. We
  // want these in radians and radians per second to use with WPILib's swerve
  // APIs.
  m_turningAbsoluteEncoder.SetPositionConversionFactor(
      kTurningEncoderPositionFactor);
  m_turningAbsoluteEncoder.SetVelocityConversionFactor(
      kTurningEncoderVelocityFactor);

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of the steering motor in the MAXSwerve Module.
  m_turningAbsoluteEncoder.SetInverted(kTurningEncoderInverted);

  // Enable PID wrap around for the turning motor. This will allow the PID
  // controller to go through 0 to get to the setpoint i.e. going from 350
  // degrees to 10 degrees will go through 0 rather than the other direction
  // which is a longer route.
  m_turningPIDController.SetPositionPIDWrappingEnabled(true);
  m_turningPIDController.SetPositionPIDWrappingMinInput(
      kTurningEncoderPositionPIDMinInput.value());
  m_turningPIDController.SetPositionPIDWrappingMaxInput(
      kTurningEncoderPositionPIDMaxInput.value());
        
  // Set the PID Controller to use the duty cycle encoder on the swerve
  // module instead of the built in NEO550 encoder.
  m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

  // Set the PID gains for the driving motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_drivingPIDController.SetP(kDrivingP);
  m_drivingPIDController.SetI(kDrivingI);
  m_drivingPIDController.SetD(kDrivingD);
  m_drivingPIDController.SetFF(kDrivingFF);
  m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

  // Set the PID gains for the turning motor. Note these are example gains, and
  // you may need to tune them for your own robot!
  m_turningPIDController.SetP(kTurningP);
  m_turningPIDController.SetI(kTurningI);
  m_turningPIDController.SetD(kTurningD);
  m_turningPIDController.SetFF(kTurningFF);
  m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

  m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
  m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
  m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
  m_turningSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());

  // Save the SPARK MAX configurations. If a SPARK MAX browns out during
  // operation, it will maintain the above configurations.
  m_drivingSparkMax.BurnFlash();
  m_turningSparkMax.BurnFlash();

  m_chassisAngularOffset = chassisAngularOffset;
  m_desiredState.angle =
      frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});
  m_drivingEncoder.SetPosition(0);
}

frc::SwerveModuleState MAXSwerveModule::GetState() const {
  return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                          m_chassisAngularOffset}};
}

frc::SwerveModulePosition MAXSwerveModule::GetPosition() const {
  return {units::meter_t{m_drivingEncoder.GetPosition()},
          units::radian_t{m_turningAbsoluteEncoder.GetPosition() -
                          m_chassisAngularOffset}};
}

void MAXSwerveModule::SetDesiredState(
    const frc::SwerveModuleState& desiredState) {
  // Apply chassis angular offset to the desired state.
  frc::SwerveModuleState correctedDesiredState{};
  correctedDesiredState.speed = desiredState.speed;
  correctedDesiredState.angle =
      desiredState.angle +
      frc::Rotation2d(units::radian_t{m_chassisAngularOffset});


  // Optimize the reference state to avoid spinning further than 90 degrees.
  frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(
      correctedDesiredState, frc::Rotation2d(units::radian_t{
                                 m_turningAbsoluteEncoder.GetPosition()}))};

  // Command driving and turning SPARKS MAX towards their respective setpoints.
  m_drivingPIDController.SetReference((double)optimizedDesiredState.speed,
                                      rev::CANSparkMax::ControlType::kVelocity);
  m_turningPIDController.SetReference(
      optimizedDesiredState.angle.Radians().value(),
      rev::CANSparkMax::ControlType::kPosition);

  m_desiredState = desiredState;
}

void MAXSwerveModule::ResetEncoders() { m_drivingEncoder.SetPosition(0); }
