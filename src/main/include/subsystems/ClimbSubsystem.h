#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include "subsystems/BaseSingleAxisSubsystem.h"
#include "utils/ConsoleLogger.h"

class ExtensionSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkBase,
                                     rev::SparkRelativeEncoder> {
   public:
    ExtensionSubsystem();

    void ResetEncoder() override;

    double GetCurrentPosition() override;

   private:
    rev::CANSparkMax m_extensionMotor{CANSparkMaxConstants::kExtensionMotorID,
                                      rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_encoder = m_extensionMotor.GetEncoder(
        rev::SparkMaxRelativeEncoder::Type::kHallSensor,
        ArmConstants::kTicksPerMotorRotation);

    SingleAxisConfig m_config = {
        .type = BaseSingleAxisSubsystem::AxisType::Linear,
        .pid = frc2::PIDController(ArmConstants::kExtenderSetP,
                                   ArmConstants::kExtenderSetI,
                                   ArmConstants::kExtenderSetD),
        .minDistance = 0,
        .maxDistance = ArmConstants::kMaxArmDistance,
        .distancePerRevolution = ArmConstants::kInPerRotation,
        .stepSize = ArmConstants::kExtenderStepSize,
        .motorMultiplier = .5,
        .pidResultMultiplier = -6.0,
        .minLimitSwitchPort = ArmConstants::kExtenderLimitSwitchPort,
        .maxLimitSwitchPort = BaseSingleAxisSubsystem::UNUSED_DIO_PORT,
        .defaultMovementSpeed = ArmConstants::kExtenderHomingSpeed};

    frc::DigitalInput min{ArmConstants::kExtenderLimitSwitchPort};
};