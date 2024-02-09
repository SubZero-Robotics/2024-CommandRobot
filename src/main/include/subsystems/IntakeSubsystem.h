#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkLowLevel.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

using namespace CANSparkMaxConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void Periodic() override;

  void SimulationPeriodic() override;

  void Stop();

  void In();

  void Out();

  bool NotePresent();

 private:
  rev::CANSparkMax m_rightIntakeSpinnyBoy{
      CANSparkMaxConstants::kLeftIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_leftIntakeSpinnyBoy{
      CANSparkMaxConstants::kRightIntakeSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DigitalInput m_beamBreak{Intakeconstants::kBeamBreakDigitalPort};
};
