#pragma once

#include <rev/CANSparkMax.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkLowLevel.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>

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
    rev::CANSparkFlex m_intakeSpinnyBoy{CANSparkMaxConstants::kIntakeSpinnyBoiId, rev::CANSparkLowLevel::MotorType::kBrushless};

    frc::DigitalInput m_beamBreak{Intakeconstants::kBeamBreakDigitalPort};
};
