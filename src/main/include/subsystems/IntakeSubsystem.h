#pragma once

#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "moduledrivers/ConnectorX.h"

#include "Constants.h"
#include "ColorConstants.h"
#include "Utils/State.h"

using namespace CANSparkMaxConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
   public:
    IntakeSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    void In(RobotState);
    void Stop();
    void Out(RobotState);

   private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    rev::CANSparkMax m_intakeSpinnyBoy{CANSparkMaxConstants::kIntakeSpinnyBoyID, rev::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_vectorSpinnyboy{CANSparkMaxConstants::kVectorSpinnyBoyID, rev::CANSparkLowLevel::MotorType::kBrushless};
};