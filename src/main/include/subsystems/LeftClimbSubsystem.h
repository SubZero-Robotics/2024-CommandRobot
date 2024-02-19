#pragma once

#include "ClimbSubsystem.h"

class LeftClimbSubsystem : public ClimbSubsystem {
 public:
  LeftClimbSubsystem()
      : ClimbSubsystem(m_config, m_motor, m_encoder, "Left climb subsystem") {}

 private:
  rev::CANSparkMax m_motor{ClimbConstants::kClimberLeftMotorId,
                           rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkRelativeEncoder m_encoder =
      m_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,
                         ClimbConstants::kTicksPerMotorRotation);

  SingleAxisConfig m_config = {
      .type = BaseSingleAxisSubsystem::AxisType::Linear,
      .pid = frc::PIDController(ClimbConstants::kClimberSetP,
                                ClimbConstants::kClimberSetI,
                                ClimbConstants::kClimberSetD),
      .minDistance = 0,
      .maxDistance = ClimbConstants::kMaxArmDistance,
      .distancePerRevolution = ClimbConstants::kInPerRotation,
      .stepSize = ClimbConstants::kClimbStepSize,
      .motorMultiplier = -1,
      .pidResultMultiplier = 1,
      .minLimitSwitchPort = BaseSingleAxisSubsystem::UNUSED_DIO_PORT,
      .maxLimitSwitchPort = BaseSingleAxisSubsystem::UNUSED_DIO_PORT,
      .defaultMovementSpeed = ClimbConstants::kClimbHomingSpeed};
};