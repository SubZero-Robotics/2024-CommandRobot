#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem.h"
#include "utils/PidMotorController.h"

using namespace ScoringConstants;

// TODO: CHANGE THIS; FOR TESTING PURPOSES ONLY
class WristSubsystem
    : public RotationalSingleAxisSubsystem<
          rev::CANSparkMax, rev::SparkPIDController, rev::SparkRelativeEncoder,
          rev::SparkAbsoluteEncoder> {
 public:
  WristSubsystem()
      : RotationalSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                      rev::SparkRelativeEncoder,
                                      rev::SparkAbsoluteEncoder>{
            "Wrist",
            upperController,
            {frc::PIDController{1, 0, 0}, 40_deg, 150_deg, 360_deg,
             10_deg_per_s, 1.0, std::nullopt, std::nullopt, false},
            0.2_m} {
    m_SpinnyBoi.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_enc.SetPositionConversionFactor(.13 * 360);
    m_absEnc.SetPositionConversionFactor(360);
    m_SpinnyBoi.BurnFlash();
  }

  void Periodic() override {
    RotationalSingleAxisSubsystem<rev::CANSparkMax, rev::SparkPIDController,
                                  rev::SparkRelativeEncoder,
                                  rev::SparkAbsoluteEncoder>::Periodic();

    // armTuner.UpdateFromShuffleboard();
  }

 private:
  // :(
  //   ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 m_config{
  //       frc::PIDController{1, 0, 0},
  //       -107_deg,
  //       263_deg,
  //       360_deg,
  //       10_deg_per_s,
  //       1.0,
  //       std::nullopt,
  //       std::nullopt,
  //       false};
  rev::CANSparkMax m_SpinnyBoi{CANSparkMaxConstants::kWristSpinnyBoiId,
                               rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkPIDController m_PidController = m_SpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_enc = m_SpinnyBoi.GetEncoder();
  rev::SparkAbsoluteEncoder m_absEnc = m_SpinnyBoi.GetAbsoluteEncoder();
  PidSettings armPidSettings = {
      .p = 0.075, .i = 0, .d = 0, .iZone = 0, .ff = 0};
  PidMotorController<rev::CANSparkMax, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      upperController{"Arm",          m_SpinnyBoi, m_enc,      m_PidController,
                      armPidSettings, &m_absEnc,   kMaxSpinRpm};
  // PidMotorControllerTuner<rev::CANSparkMax, rev::SparkPIDController,
  //                         rev::SparkRelativeEncoder,
  //                         rev::SparkAbsoluteEncoder>
  // armTuner{upperController};
};