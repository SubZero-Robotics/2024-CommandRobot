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

using namespace CANConstants;
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
  bool NotePresentUpperAmp();
  bool NotePresentLowerPodium();
  bool NotePresentLowerAmp();
  bool NotePresentUpperPodium();
  bool NotePresentCenter();
  bool NotePresentUpper();
  bool NotePresentLower();

 private:
  rev::CANSparkMax m_leftIntakeSpinnyBoy{
      CANConstants::kLeftIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_centerBeamBreak{
      IntakingConstants::kCenterBeamBreakDigitalPort};
  frc::DigitalInput m_lowerPodiumBeamBreak{
      IntakingConstants::kLowerPodiumBeamBreakDigitalPort};
  frc::DigitalInput m_upperPodiumBeamBreak{
      IntakingConstants::kUpperPodiumBeamBreakDigitalPort};
  frc::DigitalInput m_upperAmpBeamBreak{
      IntakingConstants::kUpperAmpBeamBreakDigitalPort};
  frc::DigitalInput m_lowerAmpBeamBreak{
      IntakingConstants::kLowerampBeamBreakDigitalPort};

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
