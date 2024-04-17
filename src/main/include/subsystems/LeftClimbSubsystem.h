#pragma once

#include "ClimbSubsystem.h"

class LeftClimbSubsystem : public ClimbSubsystem {
 public:
  LeftClimbSubsystem(frc::MechanismObject2d* node = nullptr)
      : ClimbSubsystem(
            "Left Climber", m_controller,
            {// Min distance
             0_in,
             // Max distance
             36_in,
             // Distance per revolution of relative encoder
             1_in / 23.1,
             // Distance per revolution of absolute encoder
             std::nullopt,
             // Default velocity
             1_fps,
             // Velocity scalar
             1.0,
             // Tolerance
             0.5_in,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             {24_in, 70_deg, 6, ColorConstants::kGreen},
             // Conversion Function
             [](units::meter_t from) {
               return std::to_string(from.convert<units::inch>().value()) +
                      " inches :(";
             }},
            node) {
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

 private:
  rev::CANSparkMax m_motor{ClimbConstants::kClimberLeftMotorId,
                           rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkPIDController m_pidController = m_motor.GetPIDController();
  rev::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
  PidSettings m_climberPidSettings = {30, 0, 0, 0, 0};
  PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      m_controller{"Left Climb Motor",
                   m_motor,
                   m_encoder,
                   m_pidController,
                   m_climberPidSettings,
                   nullptr,
                   IntakingConstants::kMaxRpm};
};