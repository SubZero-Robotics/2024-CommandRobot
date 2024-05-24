#pragma once

#include <subzero/motor/PidMotorController.cpp>
#include <subzero/singleaxis/RotationalSingleAxisSubsystem.cpp>

#include "Constants.h"

class ArmSubsystem : public RotationalSingleAxisSubsystem<SparkMaxController> {
 public:
  explicit ArmSubsystem(frc::MechanismObject2d* node = nullptr)
      : RotationalSingleAxisSubsystem<SparkMaxController>{
            "Arm",
            armController,
            {// Min distance
             ArmConstants::kHomeRotation,
             // Max distance
             ArmConstants::kMaxRotation,
             // Distance per revolution of relative encoder
             ArmConstants::kArmRelativeDistancePerRev,
             // Distance per revolution of absolute encoder
             ArmConstants::kArmAbsoluteDistancePerRev,
             // Default velocity
             ArmConstants::kDefaultVelocity,
             // Velocity scalar
             ArmConstants::kVelocityScalar,
             // Tolerance
             ArmConstants::kTolerance,
             // Min limit switch
             std::nullopt,
             // Max limit switch
             std::nullopt,
             // Reversed
             false,
             // Mechanism2d
             ArmConstants::kArmMechanism,
             // Conversion Function
             std::nullopt,

             [] { return false; }, AutoConstants::kRotationalAxisConstraints},
            ArmConstants::kArmLength,
            node} {
    m_SpinnyBoi.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  }

  void Periodic() override {
    RotationalSingleAxisSubsystem<SparkMaxController>::Periodic();
  }

 private:
  rev::CANSparkMax m_SpinnyBoi{CANConstants::kArmSpinnyBoiId,
                               rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkPIDController m_PidController = m_SpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_enc = m_SpinnyBoi.GetEncoder();
  rev::SparkAbsoluteEncoder m_absEnc = m_SpinnyBoi.GetAbsoluteEncoder();
  subzero::PidSettings armPidSettings = {
      ArmConstants::kArmP, ArmConstants::kArmI, ArmConstants::kArmD,
      ArmConstants::kArmIZone, ArmConstants::kArmFF};
  SparkMaxController armController{"Arm",
                                   m_SpinnyBoi,
                                   m_enc,
                                   m_PidController,
                                   armPidSettings,
                                   &m_absEnc,
                                   IntakingConstants::kMaxRpm};
};