#pragma once

#include "ClimbSubsystem.h"

class RightClimbSubsystem : public ClimbSubsystem {
 public:
  RightClimbSubsystem(frc::MechanismObject2d* node = nullptr)
      : ClimbSubsystem(
            "Right Climber", m_controller,
            {// Min distance
             ClimbConstants::kMinClimberDistance,
             // Max distance
             ClimbConstants::kMaxClimberDistance,
             // Distance per revolution of relative encoder
             ClimbConstants::kInPerRotation,
             // Distance per revolution of absolute encoder
             std::nullopt,
             // Default velocity
             ClimbConstants::kClimberExtendSpeed,
             // Velocity scalar
             ClimbConstants::kClimberVelocityScalar,
             // Tolerance
             ClimbConstants::kClimberTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             ClimbConstants::kRightClimberMechanism,
             // Conversion Function
             [](units::meter_t from) {
               return std::to_string(from.convert<units::inch>().value()) +
                      " inches :(";
             }},
            node) {
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

 private:
  rev::CANSparkMax m_motor{ClimbConstants::kClimberRightMotorId,
                           rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkPIDController m_pidController = m_motor.GetPIDController();
  rev::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
  PidSettings m_climberPidSettings = {
      ClimbConstants::kClimberSetP, ClimbConstants::kClimberSetI,
      ClimbConstants::kClimberSetD, ClimbConstants::kClimberSetIZone,
      ClimbConstants::kClimberSetFF};
  PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      m_controller{"Right Climb Motor",
                   m_motor,
                   m_encoder,
                   m_pidController,
                   m_climberPidSettings,
                   nullptr,
                   IntakingConstants::kMaxRpm};
};