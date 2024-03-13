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

using namespace CANSparkMaxConstants;
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

  rev::SparkPIDController m_speakerUpperPidController =
      m_speakerUpperSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_speakerUpperEnc =
      m_speakerUpperSpinnyBoi.GetEncoder();
  rev::SparkPIDController m_speakerLowerPidController =
      m_speakerLowerSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_speakerLowerEnc =
      m_speakerEncoder;
  rev::SparkPIDController m_ampUpperPidController =
      m_ampUpperSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_ampUpperEnc = m_ampUpperSpinnyBoi.GetEncoder();
  rev::SparkPIDController m_ampLowerPidController =
      m_ampLowerSpinnyBoi.GetPIDController();
  rev::SparkRelativeEncoder m_ampLowerEnc = m_ampEncoder;

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

  RevRelativePidController<rev::SparkPIDController> speakerUpperController{
      "Speaker Upper", m_speakerUpperPidController, m_speakerUpperEnc,
      speakerPidSettings, kMaxSpinRpm};

  RevRelativePidController<rev::SparkPIDController> speakerLowerController{
      "Speaker Lower", m_speakerLowerPidController, m_speakerLowerEnc,
      speakerPidSettings, kMaxSpinRpm};

  RevRelativePidController<rev::SparkPIDController> ampUpperController{
      "Amp Upper", m_ampUpperPidController, m_ampUpperEnc, ampPidSettings,
      kMaxSpinRpm};

  RevRelativePidController<rev::SparkPIDController> ampLowerController{
      "Amp Lower", m_ampLowerPidController, m_ampLowerEnc, ampPidSettings,
      kMaxSpinRpm};

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