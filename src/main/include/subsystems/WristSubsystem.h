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
            "Wrist", upperController, m_config, 0.2_m} {}

 private:
  ISingleAxisSubsystem2<units::degree>::SingleAxisConfig2 m_config = {
      .pid = frc::PIDController{1, 0, 0},
      .minDistance = 0_deg,
      .maxDistance = 100_deg,
      .distancePerRevolution = 360_deg,
      .defaultSpeed = 10_deg_per_s,
      .velocityScalar = 1.0,
      .minLimitSwitch = std::nullopt,
      .maxLimitSwitch = std::nullopt,
      .reversed = false};
  rev::CANSparkMax m_SpinnyBoi{CANSparkMaxConstants::kSpeakerUpperSpinnyBoiId,
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