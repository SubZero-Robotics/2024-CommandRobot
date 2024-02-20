#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkFlex.h>

#include "ColorConstants.h"
#include "Constants.h"

enum class ScoringDirection {
  AmpSide = 0,
  SpeakerSide,
  Subwoofer,
};

using namespace CANSparkMaxConstants;

class ScoringSubsystem : public frc2::SubsystemBase {
 public:
  ScoringSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void Stop();

  void SpinVectorSide(ScoringDirection);

  void StartScoringRamp(ScoringDirection);

  bool GetMotorAtScoringSpeed(ScoringDirection);

  /**
   * Returns true if there is no load on the motor
   */
  bool GetMotorFreeWheel(ScoringDirection);

  void SpinOutake();

 private:
  void SpinAmp();
  void SpinSpeaker();
  void SpinSubwoofer();
  bool CheckAmpSpeed();
  bool CheckSpeakerSpeed();
  bool CheckSubwooferSpeed();

  inline double MaxSpeedToRpm(double speedPercentage) {
    return ScoringConstants::kMaxSpinRpm * speedPercentage;
  }

  rev::CANSparkMax m_vectorSpinnyBoi{
      CANSparkMaxConstants::kVectorSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_ampLowerSpinnyBoi{
      CANSparkMaxConstants::kAmpLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_ampEncoder = m_ampLowerSpinnyBoi.GetEncoder(
      rev::SparkRelativeEncoder::Type::kHallSensor,
      CANSparkMaxConstants::kTicksPerMotorRotation);
  rev::CANSparkFlex m_ampUpperSpinnyBoi{
      CANSparkMaxConstants::kAmpUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_speakerLowerSpinnyBoi{
      CANSparkMaxConstants::kSpeakerLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_speakerEncoder =
      m_speakerLowerSpinnyBoi.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor,
          CANSparkMaxConstants::kTicksPerMotorRotation);
  rev::CANSparkFlex m_speakerUpperSpinnyBoi{
      CANSparkMaxConstants::kSpeakerUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  //   rev::SparkPIDController m_speakerPidController =
  //   m_speakerUpperSpinnyBoi.GetPIDController(); rev::SparkPIDController
  //   m_ampUpperPidController = m_ampUpperSpinnyBoi.GetPIDController();
  //   rev::SparkPIDController m_ampLowerPidController =
  //   m_ampLowerSpinnyBoi.GetPIDController();
};