#pragma once

#include <frc/BuiltInAccelerometer.h>
#include <frc/Notifier.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>
#include <lumyn/device/ConnectorXAnimate.h>

#include <chrono>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Constants.h"

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem() {
    m_connectorX.Connect(HAL_SerialPort_USB1);
  }

  void Periodic() override;

  void SimulationPeriodic() override;

  frc2::DeferredCommand GetDeferredFromState(StateGetter);
  frc2::CommandPtr ShowFromState(StateGetter);

  frc2::CommandPtr Intaking();
  // I really think it's outtaking with two T's...
  frc2::CommandPtr Outaking();
  frc2::CommandPtr ScoringSpeaker();
  frc2::CommandPtr ScoringAmp();
  frc2::CommandPtr ScoringSubwoofer();
  frc2::CommandPtr Loaded();
  frc2::CommandPtr Idling();
  frc2::CommandPtr Climbing();
  frc2::CommandPtr Funni();
  frc2::CommandPtr Error();
  frc2::CommandPtr AngryFace();
  frc2::CommandPtr HappyFace();
  frc2::CommandPtr BlinkingFace();
  frc2::CommandPtr SurprisedFace();
  frc2::CommandPtr AmogusFace();
  frc2::CommandPtr OwOFace();
  frc2::CommandPtr BadApple();
  frc2::CommandPtr AimbotEnabled();
  frc2::CommandPtr OnTheFlyPP();
  frc2::CommandPtr VisionNoteDetected();
  frc2::CommandPtr SuccessfulIntake();
  frc2::CommandPtr AutoScoring();

  void IdlingAsync();
  void ErrorAsync();
  void RampingAsync();

 private:
  enum class LedZone {
    LeftClimber = 0,
    Back,
    RightClimber,
    Front,
  };

  enum class EyePattern {
    Angry = 0,
    Happy,
    Blinking,
    Surprised,
    Amogus,
    OwO,
    BadApple,
  };

  void showFace(EyePattern pattern);

  lumyn::device::ConnectorXAnimate m_connectorX;

  frc::BuiltInAccelerometer m_accel;
};