#pragma once

#include "ClimbSubsystem.h"

class LeftClimbSubsystem : public ClimbSubsystem {
 public:
  LeftClimbSubsystem()
      : ClimbSubsystem("Left Climber", m_controller,
                       {// Min distance
                        0_in,
                        // Max distance
                        40_in,
                        // Distance per revolution of relative encoder
                        (1_in * (1 / 80)),
                        // Distance per revolution of absolute encoder
                        0_in,
                        // Default velocity
                        0.5_fps,
                        // Velocity scalar
                        1.0,
                        // Tolerance
                        0.5_in,
                        // Min limit switch
                        std::nullopt,
                        // Max limit switch
                        std::nullopt,
                        // Reversed
                        false}) {
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

 private:
  rev::CANSparkMax m_motor{ClimbConstants::kClimberLeftMotorId,
                           rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkPIDController m_pidController = m_motor.GetPIDController();
  rev::SparkRelativeEncoder m_encoder =
      m_motor.GetEncoder();
  PidSettings m_climberPidSettings = {// TODO: Constants
                                      .p = 0.075,
                                      .i = 0,
                                      .d = 0,
                                      .iZone = 0,
                                      .ff = 0};
  PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      m_controller{"Left Climb Motor",   m_motor, m_encoder, m_pidController,
                   m_climberPidSettings, nullptr, kMaxRpm};
};