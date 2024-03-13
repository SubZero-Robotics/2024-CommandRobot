#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem.h"

using namespace ScoringConstants;

// TODO: CHANGE THIS; FOR TESTING PURPOSES ONLY
class WristSubsystem
    : public RotationalSingleAxisSubsystem<rev::SparkPIDController,
                                           rev::SparkAbsoluteEncoder> {
 public:
  WristSubsystem()
      : RotationalSingleAxisSubsystem<rev::SparkPIDController,
                                      rev::SparkAbsoluteEncoder>{
            "Wrist", upperController, {
      frc::PIDController{1, 0, 0},
      0_deg,
      220_deg,
      360_deg,
      10_deg_per_s,
      1.0,
      std::nullopt,
      std::nullopt,
      false}, 0.2_m} {}

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
  rev::SparkAbsoluteEncoder m_enc = m_SpinnyBoi.GetAbsoluteEncoder();
  PidSettings speakerPidSettings = {.p = ScoringPID::kSpeakerP,
                                    .i = ScoringPID::kSpeakerI,
                                    .d = ScoringPID::kSpeakerD,
                                    .iZone = ScoringPID::kSpeakerIZone,
                                    .ff = ScoringPID::kSpeakerFF};
  RevAbsolutePidController<rev::SparkPIDController> upperController{
      "Speaker Upper", m_PidController, m_enc, speakerPidSettings, kMaxSpinRpm};
};