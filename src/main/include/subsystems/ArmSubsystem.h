#pragma once

#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem2.h"
#include "utils/PidMotorController.h"

class ArmSubsystem : public RotationalSingleAxisSubsystem<
                         rev::CANSparkMax, rev::SparkPIDController,
                         rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder> {
 public:
  ArmSubsystem(frc::MechanismObject2d* node = nullptr)
      : RotationalSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                      rev::SparkRelativeEncoder,
                                      rev::SparkAbsoluteEncoder>{
            "Arm",
            armController,
            {// Min distance
             10_deg,
             // Max distance
             190_deg,
             // Distance per revolution of relative encoder
             (360_deg * (1 / 8.75)),
             // Distance per revolution of absolute encoder
             360_deg,
             // Default velocity
             10_deg_per_s,
             // Velocity scalar
             1.0,
             // Tolerance
             2_deg,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             {0.2_m, 110_deg, 6, ColorConstants::kBlue}},
            0.2_m,
            node} {
    m_SpinnyBoi.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void Periodic() override {
    RotationalSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                  rev::SparkRelativeEncoder,
                                  rev::SparkAbsoluteEncoder>::Periodic();
  }

 private:
  rev::CANSparkMax m_SpinnyBoi{CANConstants::kArmSpinnyBoiId,
                               rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkPIDController m_PidController = m_SpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_enc = m_SpinnyBoi.GetEncoder();
  rev::SparkAbsoluteEncoder m_absEnc = m_SpinnyBoi.GetAbsoluteEncoder();
  PidSettings armPidSettings = {ArmConstants::kArmP, ArmConstants::kArmI,
                                ArmConstants::kArmD, ArmConstants::kArmIZone,
                                ArmConstants::kArmFF};
  PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      armController{"Arm",          m_SpinnyBoi, m_enc,  m_PidController,
                    armPidSettings, &m_absEnc,   IntakingConstants::kMaxRpm};
  // PidMotorControllerTuner<rev::CANSparkMax, rev::SparkPIDController,
  //                         rev::SparkRelativeEncoder,
  //                         rev::SparkAbsoluteEncoder>
  // armTuner{upperController};
};