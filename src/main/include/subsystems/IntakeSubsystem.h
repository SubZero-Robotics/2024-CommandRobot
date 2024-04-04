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
  bool NotePresentUpperCenter();
  bool NotePresentLowerCenter();
  bool NotePresentUpper();
  bool NotePresentLower();
  bool NotePresentUpperAll();

 private:
  rev::CANSparkMax m_leftIntakeSpinnyBoy{
      CANConstants::kLeftIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_centerLowerBeamBreak{
      IntakingConstants::kCenterLowerBeamBreakDigitalPort};
  frc::DigitalInput m_centerUpperBeamBreak{
      IntakingConstants::kCenterUpperBeamBreakDigitalPort};
  frc::DigitalInput m_lowerPodiumBeamBreak{
      IntakingConstants::kLowerPodiumBeamBreakDigitalPort};
  frc::DigitalInput m_upperPodiumBeamBreak{
      IntakingConstants::kUpperPodiumBeamBreakDigitalPort};
  frc::DigitalInput m_upperAmpBeamBreak{
      IntakingConstants::kUpperAmpBeamBreakDigitalPort};
  frc::DigitalInput m_lowerAmpBeamBreak{
      IntakingConstants::kLowerampBeamBreakDigitalPort};
};
