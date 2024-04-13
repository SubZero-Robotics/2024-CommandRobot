#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkFlex.h>

#include <map>
#include <string>

#include "ColorConstants.h"
#include "Constants.h"
#include "utils/PidMotorControllerPair.h"

enum class ScoringDirection { AmpSide = 0, PodiumSide, Subwoofer };

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

  void SpinAmp(double upperPercentage = ScoringConstants::kAmpUpperSpeed,
               double lowerPercentage = ScoringConstants::kAmpLowerSpeed);

 private:
  void SpinSpeaker();
  void SpinSubwoofer();
  bool CheckAmpSpeed();
  bool CheckSpeakerSpeed();
  bool CheckSubwooferSpeed();

  inline double MaxSpeedToRpm(double speedPercentage) {
    return ScoringConstants::kMaxSpinRpm.value() * speedPercentage;
  }

  rev::CANSparkMax m_vectorSpinnyBoi{
      CANConstants::kVectorSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_ampLowerSpinnyBoi{
      CANConstants::kAmpLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_ampUpperSpinnyBoi{
      CANConstants::kAmpUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_speakerLowerSpinnyBoi{
      CANConstants::kSpeakerLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_speakerUpperSpinnyBoi{
      CANConstants::kSpeakerUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::SparkPIDController m_speakerUpperPidController =
      m_speakerUpperSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_speakerUpperEnc =
      m_speakerUpperSpinnyBoi.GetEncoder();
  rev::SparkPIDController m_speakerLowerPidController =
      m_speakerLowerSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_speakerLowerEnc =
      m_speakerLowerSpinnyBoi.GetEncoder();
  rev::SparkPIDController m_ampUpperPidController =
      m_ampUpperSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_ampUpperEnc = m_ampUpperSpinnyBoi.GetEncoder();
  rev::SparkPIDController m_ampLowerPidController =
      m_ampLowerSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_ampLowerEnc = m_ampLowerSpinnyBoi.GetEncoder();

  PidSettings speakerPidSettings = {
      .p = ScoringConstants::ScoringPID::kSpeakerP,
      .i = ScoringConstants::ScoringPID::kSpeakerI,
      .d = ScoringConstants::ScoringPID::kSpeakerD,
      .iZone = ScoringConstants::ScoringPID::kSpeakerIZone,
      .ff = ScoringConstants::ScoringPID::kSpeakerFF};

  PidSettings ampPidSettings = {
      .p = ScoringConstants::ScoringPID::kAmpP,
      .i = ScoringConstants::ScoringPID::kAmpI,
      .d = ScoringConstants::ScoringPID::kAmpD,
      .iZone = ScoringConstants::ScoringPID::kAmpIZone,
      .ff = ScoringConstants::ScoringPID::kAmpFF};

  PidMotorController<rev::CANSparkFlex, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      speakerUpperController{"Speaker Upper",
                             m_speakerUpperSpinnyBoi,
                             m_speakerUpperEnc,
                             m_speakerUpperPidController,
                             speakerPidSettings,
                             nullptr,
                             ScoringConstants::kMaxSpinRpm};

  PidMotorController<rev::CANSparkFlex, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      speakerLowerController{"Speaker Lower",
                             m_speakerLowerSpinnyBoi,
                             m_speakerLowerEnc,
                             m_speakerLowerPidController,
                             speakerPidSettings,
                             nullptr,
                             ScoringConstants::kMaxSpinRpm};

  PidMotorController<rev::CANSparkFlex, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      ampUpperController{"Amp Upper",
                         m_ampUpperSpinnyBoi,
                         m_ampUpperEnc,
                         m_ampUpperPidController,
                         ampPidSettings,
                         nullptr,
                         ScoringConstants::kMaxSpinRpm};

  PidMotorController<rev::CANSparkFlex, rev::SparkPIDController,
                     rev::SparkRelativeEncoder, rev::SparkAbsoluteEncoder>
      ampLowerController{"Amp Lower",
                         m_ampLowerSpinnyBoi,
                         m_ampLowerEnc,
                         m_ampLowerPidController,
                         ampPidSettings,
                         nullptr,
                         ScoringConstants::kMaxSpinRpm};

  //   double m_ampUpperVelocity;
  //   double m_ampLowerVelocity;
};