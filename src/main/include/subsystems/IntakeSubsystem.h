#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>

#include "Constants.h"
#include "subsystems/ScoringSubsystem.h"

using namespace CANSparkMaxConstants;
using namespace IntakingConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void Periodic() override;

  void SimulationPeriodic() override;

  void Stop();

  void In(double intakeSpeed = IntakingConstants::kIntakeSpeed);

  void Out(double outakeSpeed = IntakingConstants::kOutakeSpeed);

  void Feed(ScoringDirection direction);

  bool NotePresent();
  bool NotePresentUpper();
  bool NotePresentLower();

 private:
  rev::CANSparkMax m_rightIntakeSpinnyBoy{
      CANSparkMaxConstants::kRightIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftIntakeSpinnyBoy{
      CANSparkMaxConstants::kLeftIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_upperBeamBreak{
      IntakingConstants::kUpperBeamBreakDigitalPort};
  frc::DigitalInput m_lowerBeamBreak{
      IntakingConstants::kLowerBeamBreakDigitalPort};

  //   rev::SparkPIDController m_rightIntakeSpinnyBoyPID =
  //       m_rightIntakeSpinnyBoy.GetPIDController();
  //   rev::SparkPIDController m_leftIntakeSpinnyBoyPID =
  //       m_leftIntakeSpinnyBoy.GetPIDController();

  //   PidSettings intakePidSettings = {.p = IntakingPID::kIntakingP,
  //                                    .i = IntakingPID::kIntakingI,
  //                                    .d = IntakingPID::kIntakingD,
  //                                    .iZone = IntakingPID::kIntakingIZone,
  //                                    .ff = IntakingPID::kIntakingFF};

  //   PidMotorControllerPair intakeMotors{"Intake", m_leftIntakeSpinnyBoyPID,
  //                                       m_rightIntakeSpinnyBoyPID,
  //                                       intakePidSettings, kMaxRpm};

  //   PidMotorControllerPairTuner intakeTuner{intakeMotors};
};
