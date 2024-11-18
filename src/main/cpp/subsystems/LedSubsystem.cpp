#include "subsystems/LedSubsystem.h"

#include <frc/Timer.h>
#include <subzero/constants/ColorConstants.h>
#include <subzero/logging/ConsoleLogger.h>

#include "Constants.h"

using namespace subzero;
using namespace ConnectorX;
using namespace LEDConstants;

void LedSubsystem::Periodic() {
  if (m_accel.GetY() <= -kAccelThreshold) {
    showFace(EyePattern::OwO);
  }
}

void LedSubsystem::SimulationPeriodic() {}

frc2::DeferredCommand LedSubsystem::GetDeferredFromState(
    StateGetter stateGetter) {
  return frc2::DeferredCommand(
      [this, stateGetter] { return ShowFromState(stateGetter); }, {this});
}

frc2::CommandPtr LedSubsystem::ShowFromState(StateGetter stateGetter) {
  auto state = stateGetter();

  switch (state) {
    case RobotState::Intaking:
      return Intaking();
    case RobotState::ScoringSpeaker:
      return ScoringSpeaker();
    case RobotState::ScoringAmp:
      return ScoringAmp();
    case RobotState::Loaded:
      return Loaded();
    case RobotState::Manual:
      return Idling();
    default:
      return frc2::InstantCommand([] {}).ToPtr();
  }
}

frc2::CommandPtr LedSubsystem::Intaking() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Intaking");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kRed, 60_ms);
           m_connectorX.SetGroupAnimation("frontback",
                                          lumyn::led::Animation::Blink,
                                          ColorConstants::kRed, 400_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Outaking() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Outaking");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kRed, 60_ms, true);
           m_connectorX.SetGroupAnimation("frontback",
                                          lumyn::led::Animation::Blink,
                                          ColorConstants::kRed, 400_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringSpeaker");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kPurple, 100_ms);
           m_connectorX.SetGroupAnimation("frontback",
                                          lumyn::led::Animation::SineRoll,
                                          ColorConstants::kRed, 40_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringAmp");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kTeal, 100_ms);
           m_connectorX.SetGroupAnimation(
               "frontback", lumyn::led::Animation::Fill, ColorConstants::kBlack,
               1000_ms, false, true);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSubwoofer() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringSubwoofer");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kYellow, 100_ms);
           m_connectorX.SetGroupAnimation(
               "frontback", lumyn::led::Animation::Fill, ColorConstants::kBlack,
               1000_ms, false, true);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Loaded() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Loaded");
           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Breathe,
                                          ColorConstants::kGreen, 15_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Idling() {
  return frc2::InstantCommand([this] { IdlingAsync(); }).ToPtr();
}

void LedSubsystem::IdlingAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Idling");
  m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Breathe,
                                 ColorConstants::kBlue, 15_ms);
}

frc2::CommandPtr LedSubsystem::Climbing() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Climbing");
           m_connectorX.SetGroupAnimation("climbers",
                                          lumyn::led::Animation::Chase,
                                          ColorConstants::kGreen, 120_ms);
           m_connectorX.SetGroupAnimation("frontback",
                                          lumyn::led::Animation::SineRoll,
                                          ColorConstants::kGreen, 50_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Funni() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Funni");
           m_connectorX.SetGroupAnimation("all",
                                          lumyn::led::Animation::RainbowFade,
                                          ColorConstants::kGreen, 15_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Error() {
  return frc2::InstantCommand([this] { ErrorAsync(); }).ToPtr();
}

void LedSubsystem::ErrorAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Error");
  m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Blink,
                                 ColorConstants::kRed, 750_ms);
}

frc2::CommandPtr LedSubsystem::AngryFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Angry");
           showFace(EyePattern::Angry);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::HappyFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Happy");
           showFace(EyePattern::Happy);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::BlinkingFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Blinking");
           showFace(EyePattern::Blinking);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::SurprisedFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Surprised");
           showFace(EyePattern::Surprised);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::AmogusFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Amogus");
           showFace(EyePattern::Amogus);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::BadApple() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Bad Apple");
           showFace(EyePattern::BadApple);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::OwOFace() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "OwO");
           showFace(EyePattern::OwO);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::AimbotEnabled() {
  // Acid green

  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n",
                                 "AimbotEnabled");
           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Blink,
                                          ColorConstants::kOrange, 600_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::OnTheFlyPP() {
  // Blue SetAll for duration of on the fly

  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "OnTheFlyPP");

           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Chase,
                                          ColorConstants::kPurple, 60_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::VisionNoteDetected() {
  // Chases orange
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "VisionNoteDetected");

           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Chase,
                                          ColorConstants::kOrange, 120_ms);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::SuccessfulIntake() {
  // Flash LEDs green once
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "SuccessfulIntake");
           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Fill,
                                          ColorConstants::kGreen, 1000_ms,
                                          false, true);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::AutoScoring() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n",
                                 "AutoScoring");

           m_connectorX.SetGroupAnimation("all", lumyn::led::Animation::Blink,
                                          ColorConstants::kRed, 250_ms);
         })
      .ToPtr();
}

void LedSubsystem::RampingAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n", "Ramping");

  m_connectorX.SetGroupAnimation("climbers", lumyn::led::Animation::Chase,
                                 ColorConstants::kOrange, 40_ms);
  m_connectorX.SetGroupAnimation("frontback", lumyn::led::Animation::SineRoll,
                                 ColorConstants::kOrange, 30_ms);
}

void LedSubsystem::showFace(EyePattern pattern) {
  switch (pattern) {
    case EyePattern::Amogus:
      m_connectorX.SetImageSequence("front-matrix", "amogus",
                                    ColorConstants::kOrange);
      break;
    case EyePattern::Angry:
      m_connectorX.SetImageSequence("front-matrix", "angry_eyes",
                                    ColorConstants::kOrange);
      break;
    case EyePattern::Blinking:
      m_connectorX.SetImageSequence("front-matrix", "blinking_eyes",
                                    ColorConstants::kOrange);
      break;
    case EyePattern::OwO:
      m_connectorX.SetImageSequence("front-matrix", "owo_eyes",
                                    ColorConstants::kOrange);
      break;
    case EyePattern::Surprised:
      m_connectorX.SetImageSequence("front-matrix", "surprised_eyes",
                                    ColorConstants::kOrange);
      break;
    case EyePattern::BadApple:
      m_connectorX.SetImageSequence("front-matrix", "bad-apple_32x8",
                                    ColorConstants::kOrange);
      break;
  }
}