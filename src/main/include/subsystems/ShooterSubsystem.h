#include "Constants.h"
#include "ColorConstants.h"
#include <rev/CANSparkFlex.h>

using namespace CANSparkMaxConstants;

class ShooterSubsystem : public frc2::SubsystemBase {
   public:
    ShooterSubsystem();

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
    rev::CANSparkFlex m_SpinnyBoi1{52, rev::CANSparkLowLevel::MotorType::kBrushless};
    rev::CANSparkFlex m_SpinnyBoi2{54, rev::CANSparkLowLevel::MotorType::kBrushless};
};