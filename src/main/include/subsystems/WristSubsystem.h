#include "Constants.h"
#include "subsystems/BaseSingleAxisSubsystem.h"

using namespace ScoringConstants;

// TODO: CHANGE THIS; FOR TESTING PURPOSES ONLY
class WristSubsystem
    : public RotationalSingleAxisSubsystem<rev::SparkPIDController,
                                           rev::SparkRelativeEncoder> {
 public:
  WristSubsystem()
      : RotationalSingleAxisSubsystem<rev::SparkPIDController,
                                      rev::SparkRelativeEncoder>{
            "Wrist", upperController, m_config, &m_minLimit, std::nullopt} {}

 private:
  SingleAxisConfig2<units::degree_t, units::degrees_per_second_t> m_config = {
      .pid = frc::PIDController{1, 0, 0},
      .minDistance = 0_deg,
      .maxDistance = 100_deg,
      .distancePerRevolution = 10_deg,
      .velocityScalar = 1.0,
      .pidResultMultiplier = 1.0,
      .reversed = false};
  frc::DigitalInput m_minLimit = frc::DigitalInput{4};
  rev::CANSparkMax m_SpinnyBoi{CANSparkMaxConstants::kSpeakerUpperSpinnyBoiId,
                               rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkPIDController m_PidController = m_SpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_enc = m_SpinnyBoi.GetEncoder();
  PidSettings speakerPidSettings = {.p = ScoringPID::kSpeakerP,
                                    .i = ScoringPID::kSpeakerI,
                                    .d = ScoringPID::kSpeakerD,
                                    .iZone = ScoringPID::kSpeakerIZone,
                                    .ff = ScoringPID::kSpeakerFF};
  PidMotorController<rev::SparkPIDController, rev::SparkRelativeEncoder>
      upperController{"Speaker Upper", m_PidController, m_enc,
                      speakerPidSettings, kMaxSpinRpm};
};