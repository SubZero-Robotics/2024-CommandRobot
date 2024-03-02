#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <subsystems/BaseSingleAxisSubsystem.h>

#include "subsystems/BaseSingleAxisSubsystem.h"

using namespace ArmGuideConstants;

class AmpScoringGuideSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkMax,
                                     rev::SparkAbsoluteEncoder> {
 public:
 private:
  rev::CANSparkMax m_motor{CANSparkMaxConstants::kLeftAmpGuideId,
                           rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  rev::SparkAbsoluteEncoder m_encoder = m_motor.GetAbsoluteEncoder(
      rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

  SingleAxisConfig m_config = {
      .type = BaseSingleAxisSubsystem::AxisType::Rotational,
      .pid = frc::PIDController(kArmP, kArmI, kArmD),
      .minDistance = 0,
      .maxDistance = kMaxArmDistance,
      .distancePerRevolution = kDistancePerRevolution,
      .motorMultiplier = 1,
      .pidResultMultiplier = 1,
      .minLimitSwitchPort = kMinLimitSwitchPort,
      .maxLimitSwitchPort = kMaxLimitSwitchPort,
      .defaultMovementSpeed = kSpeed};
};