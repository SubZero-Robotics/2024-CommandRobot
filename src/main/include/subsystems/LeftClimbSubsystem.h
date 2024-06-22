#pragma once

#include <functional>

#include "ClimbSubsystem.h"

class LeftClimbSubsystem : public ClimbSubsystem {
 public:
  explicit LeftClimbSubsystem(std::function<bool()> ignoreLimit,
                              frc::MechanismObject2d* node = nullptr)
      : ClimbSubsystem(
            "Left Climber",
            frc::RobotBase::IsReal()
                ? dynamic_cast<IPidMotorController&>(m_controller)
                : dynamic_cast<IPidMotorController&>(simClimbController),
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
             ClimbConstants::kLeftClimberMechanism,
             // Conversion Function
             [](units::meter_t from) {
               return std::to_string(from.convert<units::inch>().value()) +
                      " inches :(";
             },
             ignoreLimit, AutoConstants::kLinearAxisConstraints},
            node) {
    m_motor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

 private:
  rev::CANSparkMax m_motor{ClimbConstants::kClimberLeftMotorId,
                           rev::CANSparkMax::MotorType::kBrushless};
  rev::SparkPIDController m_pidController = m_motor.GetPIDController();
  rev::SparkRelativeEncoder m_encoder = m_motor.GetEncoder();
  PidSettings m_climberPidSettings = {
      ClimbConstants::kClimberSetP, ClimbConstants::kClimberSetI,
      ClimbConstants::kClimberSetD, ClimbConstants::kClimberSetIZone,
      ClimbConstants::kClimberSetFF};
  SparkMaxController m_controller{"Left Climb Motor",
                                  m_motor,
                                  m_encoder,
                                  m_pidController,
                                  m_climberPidSettings,
                                  nullptr,
                                  IntakingConstants::kMaxRpm};
  subzero::SimPidMotorController simClimbController{
      "Sim Left Climb", m_climberPidSettings, IntakingConstants::kMaxRpm};
};