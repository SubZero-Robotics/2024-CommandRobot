#pragma once

#include <frc/controller/PIDController.h>
#include <memory>
#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <subsystems/BaseSingleAxisSubsystem.h>
#include <utils/ConsoleLogger.h>

class ClimbSubsystem
    : public BaseSingleAxisSubsystem<rev::CANSparkBase,
                                     rev::SparkRelativeEncoder> {
   public:
    ClimbSubsystem(SingleAxisConfig pConfig, std::string &name, int motorID) : 
                                    BaseSingleAxisSubsystem(pConfig, nullptr, nullptr, nullptr, nullptr, name) {
        m_climbMotor = std::make_unique(motorID, rev::CANSparkLowLevel::MotorType::kBrushless);
        _motor = m_climbMotor.get();

        m_encoder = std::move(m_climbMotor->GetEncoder(
            rev::SparkMaxRelativeEncoder::Type::kHallSensor,
            ClimbConstants::kTicksPerMotorRotation));

        _enc = &m_encoder;
    }

    void ResetEncoder() override;

    double GetCurrentPosition() override;

   private:
    std::unique_ptr<rev::CANSparkMax> m_climbMotor;
    rev::SparkRelativeEncoder &m_encoder;

    frc::DigitalInput min{ClimbConstants::kClimberLeftLimitSwitchPort};

};