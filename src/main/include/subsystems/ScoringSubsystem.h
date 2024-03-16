#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkFlex.h>

#include <map>
#include <string>

#include "ColorConstants.h"
#include "Constants.h"
#include "utils/PidMotorControllerPair.h"

enum class ScoringDirection {
  AmpSide = 0,
  SpeakerSide,
  Subwoofer,
};

using namespace CANConstants;
using namespace ScoringConstants;

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

  void SpinAmp(double upperPercentage = kAmpUpperSpeed,
               double lowerPercentage = kAmpLowerSpeed);

 private:
  void SpinSpeaker();
  void SpinSubwoofer();
  bool CheckAmpSpeed();
  bool CheckSpeakerSpeed();
  bool CheckSubwooferSpeed();

  inline double MaxSpeedToRpm(double speedPercentage) {
    return kMaxSpinRpm.value() * speedPercentage;
  }

  rev::CANSparkMax m_vectorSpinnyBoi{
      CANConstants::kVectorSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_ampLowerSpinnyBoi{
      CANConstants::kAmpLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_ampEncoder = m_ampLowerSpinnyBoi.GetEncoder(
      rev::SparkRelativeEncoder::Type::kHallSensor,
      CANConstants::kTicksPerMotorRotation);
  rev::CANSparkFlex m_ampUpperSpinnyBoi{
      CANConstants::kAmpUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkFlex m_speakerLowerSpinnyBoi{
      CANConstants::kSpeakerLowerSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::SparkRelativeEncoder m_speakerEncoder =
      m_speakerLowerSpinnyBoi.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor,
          CANConstants::kTicksPerMotorRotation);
  rev::CANSparkFlex m_speakerUpperSpinnyBoi{
      CANConstants::kSpeakerUpperSpinnyBoiId,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::SparkPIDController m_speakerUpperPidController =
      m_speakerUpperSpinnyBoi.GetPIDController();
  rev::SparkPIDController m_speakerLowerPidController =
      m_speakerLowerSpinnyBoi.GetPIDController();
  rev::SparkPIDController m_ampUpperPidController =
      m_ampUpperSpinnyBoi.GetPIDController();
  rev::SparkPIDController m_ampLowerPidController =
      m_ampLowerSpinnyBoi.GetPIDController();

  PidSettings speakerPidSettings = {.p = ScoringPID::kSpeakerP,
                                    .i = ScoringPID::kSpeakerI,
                                    .d = ScoringPID::kSpeakerD,
                                    .iZone = ScoringPID::kSpeakerIZone,
                                    .ff = ScoringPID::kSpeakerFF};

  PidSettings ampPidSettings = {.p = ScoringPID::kAmpP,
                                .i = ScoringPID::kAmpI,
                                .d = ScoringPID::kAmpD,
                                .iZone = ScoringPID::kAmpIZone,
                                .ff = ScoringPID::kAmpFF};

  PidMotorController speakerUpperController{"Speaker Upper",
                                            m_speakerUpperPidController,
                                            speakerPidSettings, kMaxSpinRpm};

  PidMotorController speakerLowerController{"Speaker Lower",
                                            m_speakerLowerPidController,
                                            speakerPidSettings, kMaxSpinRpm};

  PidMotorController ampUpperController{"Amp Upper", m_ampUpperPidController,
                                        ampPidSettings, kMaxSpinRpm};

  PidMotorController ampLowerController{"Amp Lower", m_ampLowerPidController,
                                        ampPidSettings, kMaxSpinRpm};

  //   PidMotorControllerPair speakerPidPair{"Speaker", speakerUpperController,
  //                                         speakerLowerController};

  //   PidMotorControllerPair ampPidPair{"Amp", ampUpperController,
  //                                     ampLowerController};

  // PidMotorControllerPairTuner speakerTuner{speakerSideMotors};
  //   PidMotorControllerPairTuner speakerTuner{speakerPidPair};
  //   PidMotorControllerPairTuner ampTuner{ampPidPair};
  //   PidMotorControllerTuner speakerUpperTuner{speakerUpperController};

  double m_ampUpperVelocity;
  double m_ampLowerVelocity;
};