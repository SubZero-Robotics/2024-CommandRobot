#pragma once

#include <rev/CANSparkMax.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "moduledrivers/ConnectorX.h"

#include "constants.h"
#include "ColorConstants.h"

using namespace CANSparkMaxConstants;

class IntakeSubsystem : public frc2::SubsystemBase {
   public:
    IntakeSubsystem(ConnectorX::ConnectorXBoard* subsystem);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    void Out();
    void In();
    void Stop();

   private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
    ConnectorX::ConnectorXBoard* m_ledSubsystem;
    rev::CANSparkMax m_intakeSpinnyBoy{CANSparkMaxConstants::kIntakeSpinnyBoyID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
};