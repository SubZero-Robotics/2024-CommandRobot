#pragma once

#include <frc/controller/PIDController.h>
#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <subsystems/BaseSingleAxisSubsystem.h>
#include <utils/ConsoleLogger.h>

class ClimbSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkBase,
                                     rev::SparkRelativeEncoder> {
   public:
    ClimbSubsystem(int motorID) ;

    void ResetEncoder() override;

    double GetCurrentPosition() override;

   private:
    // Put this inside of the constructor and have motorID as a parameter
    rev::CANSparkMax m_extensionMotor{motorID,
                                      rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkMaxRelativeEncoder m_encoder = m_extensionMotor.GetEncoder(
        rev::SparkMaxRelativeEncoder::Type::kHallSensor,
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
        .motorMultiplier = .5,
        .pidResultMultiplier = -6.0,
        .minLimitSwitchPort = ClimbConstants::kClimberLeftLimitSwitchPort,
        .maxLimitSwitchPort = BaseSingleAxisSubsystem::UNUSED_DIO_PORT,
        .defaultMovementSpeed = ClimbConstants::kClimbHomingSpeed};

    frc::DigitalInput min{ClimbConstants::kClimberLeftLimitSwitchPort};
};