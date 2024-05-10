#pragma once

#include <frc/DigitalInput.h>
#include <frc/filter/Debouncer.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <hal/SimDevice.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkPIDController.h>

#include "Constants.h"
#include "subsystems/ScoringSubsystem.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem() : m_simDevice{"IntakeSubsystem"} {
    if (m_simDevice) {
      m_notePresent = m_simDevice.CreateBoolean("Note present", 0, false);
    }
  }

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
  frc::Debouncer m_centerLowerDb{IntakingConstants::kDebounceTime,
                               IntakingConstants::kDebounceType};
  frc::DigitalInput m_centerUpperBeamBreak{
      IntakingConstants::kCenterUpperBeamBreakDigitalPort};
  frc::Debouncer m_centerUpperDb{IntakingConstants::kDebounceTime,
                                 IntakingConstants::kDebounceType};
  frc::DigitalInput m_lowerPodiumBeamBreak{
      IntakingConstants::kLowerPodiumBeamBreakDigitalPort};
  frc::Debouncer m_lowerPodiumDb{IntakingConstants::kDebounceTime,
                                 IntakingConstants::kDebounceType};
  frc::DigitalInput m_upperPodiumBeamBreak{
      IntakingConstants::kUpperPodiumBeamBreakDigitalPort};
  frc::Debouncer m_upperPodiumDb{IntakingConstants::kDebounceTime,
                                 IntakingConstants::kDebounceType};
  frc::DigitalInput m_upperAmpBeamBreak{
      IntakingConstants::kUpperAmpBeamBreakDigitalPort};
  frc::Debouncer m_upperAmpDb{IntakingConstants::kDebounceTime,
                              IntakingConstants::kDebounceType};
  frc::DigitalInput m_lowerAmpBeamBreak{
      IntakingConstants::kLowerampBeamBreakDigitalPort};
  frc::Debouncer m_lowerAmpDb{IntakingConstants::kDebounceTime,
                              IntakingConstants::kDebounceType};

  hal::SimDevice m_simDevice;
  hal::SimBoolean m_notePresent;
};
