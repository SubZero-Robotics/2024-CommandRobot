#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem.h"

class ArmSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkMax,
                                     rev::SparkMaxAbsoluteEncoder> {
 public:
  ArmSubsystem();

  // Wrist has zero offset set in SparkMax
  void ResetEncoder() override;

  double GetCurrentPosition() override;

 private:
  rev::CANSparkMax m_wristMotor{CANSparkMaxConstants::kArmRotationMotorID,
                                rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxAbsoluteEncoder m_encoder = m_wristMotor.GetAbsoluteEncoder(
      rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

  SingleAxisConfig m_config = {
      .type = BaseSingleAxisSubsystem::AxisType::Rotational,
      .pid =
          frc::PIDController(ArmConstants::kWristSetP, ArmConstants::kWristSetI,
                             ArmConstants::kWristSetD),
      .minDistance = 0,
      .maxDistance = ArmConstants::kWristDegreeLimit,
      .distancePerRevolution = 360.0,
      .stepSize = ArmConstants::kWristStepSize,
      .motorMultiplier = -1.0,
      .pidResultMultiplier = 0.66,
      .minLimitSwitchPort = ArmConstants::kWristLimitSwitchPort,
      .maxLimitSwitchPort = BaseSingleAxisSubsystem::UNUSED_DIO_PORT,
      .defaultMovementSpeed = -ArmConstants::kWristHomingSpeed};

  frc::DigitalInput min{ArmConstants::kWristLimitSwitchPort};
};