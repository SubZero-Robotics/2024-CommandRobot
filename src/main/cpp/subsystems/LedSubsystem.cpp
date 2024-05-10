#include "subsystems/LedSubsystem.h"

#include <frc/Timer.h>

#include "ColorConstants.h"
#include "Constants.h"
#include "utils/ConsoleLogger.h"

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
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Chase, false, 60, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Chase, false, 60, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Blink, false, 400, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kRed, PatternType::Blink,
                                    false, 400, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Outaking() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Outaking");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Chase, false, 60, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Chase, false, 60, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kRed, PatternType::Blink,
                                    false, 400, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Blink, false, 400, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
           delaySeconds(kConnectorXDelay);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringSpeaker");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kPurple, PatternType::Chase, false, 100, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kPurple, PatternType::Chase, false, 100, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kRed, PatternType::SineRoll,
                                    false, 40, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::SineRoll, false, 40, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringAmp");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kTeal, PatternType::Chase, false, 100, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kTeal, PatternType::Chase, false, 100, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kBlack, PatternType::SetAll,
                                    true, -1, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kBlack, PatternType::SetAll, true, -1, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSubwoofer() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "ScoringSubwoofer");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kYellow, PatternType::Chase, false, 100, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kYellow, PatternType::Chase, false, 100, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kBlack, PatternType::SetAll,
                                    true, -1, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kBlack, PatternType::SetAll, true, -1, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Loaded() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Loaded");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIdleLedPort,
               ColorConstants::kGreen, PatternType::Breathe, false, 15, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIdleLedPort,
               ColorConstants::kGreen, PatternType::Breathe, false, 15, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIdleLedPort,
                                    ColorConstants::kGreen,
                                    PatternType::Breathe, false, 15, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Front, LedConstants::kIdleLedPort,
                                    ColorConstants::kGreen,
                                    PatternType::Breathe, false, 15, false);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Idling() {
  return frc2::InstantCommand([this] { IdlingAsync(); }).ToPtr();
}

void LedSubsystem::IdlingAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Idling");
  setZoneColorPatternAsync(LedZone::LeftClimber, LedConstants::kIdleLedPort,
                           ColorConstants::kBlue, PatternType::Breathe, false,
                           15, false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::RightClimber, LedConstants::kIdleLedPort,
                           ColorConstants::kBlue, PatternType::Breathe, false,
                           15, false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Back, LedConstants::kIdleLedPort,
                           ColorConstants::kBlue, PatternType::Breathe, false,
                           15, false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Front, LedConstants::kIdleLedPort,
                           ColorConstants::kBlue, PatternType::Breathe, false,
                           15, false);
  delaySeconds(kConnectorXDelay);
  syncAllZones();
}

frc2::CommandPtr LedSubsystem::Climbing() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s",
                                 "Climbing");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::Chase, false, 120, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::Chase, false, 120, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kGreen,
                                    PatternType::SineRoll, false, 50, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::SineRoll, false, 50, false);
           delaySeconds(kConnectorXDelay);
           m_connectorX.syncZones(
               LedConstants::kIntakeLedPort,
               {
                   static_cast<uint8_t>(LedZone::LeftClimber),
                   static_cast<uint8_t>(LedZone::RightClimber),
               });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Funni() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Funni");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::RGBFade, false, 15, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::RGBFade, false, 15, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kRed, PatternType::RGBFade,
                                    false, 15, false);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Error() {
  return frc2::InstantCommand([this] { ErrorAsync(); }).ToPtr();
}

void LedSubsystem::ErrorAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s", "Error");
  setZoneColorPatternAsync(LedZone::LeftClimber, LedConstants::kIntakeLedPort,
                           ColorConstants::kRed, PatternType::Blink, false, 750,
                           false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::RightClimber, LedConstants::kIntakeLedPort,
                           ColorConstants::kRed, PatternType::Blink, false, 750,
                           false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                           ColorConstants::kRed, PatternType::Blink, false, 750,
                           false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Front, LedConstants::kIntakeLedPort,
                           ColorConstants::kRed, PatternType::Blink, false, 750,
                           false);
  delaySeconds(kConnectorXDelay);
  m_connectorX.syncZones(LedConstants::kIntakeLedPort,
                         {
                             static_cast<uint8_t>(LedZone::LeftClimber),
                             static_cast<uint8_t>(LedZone::RightClimber),
                         });
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
           setZoneColorPatternAsync(LedZone::Back, LedPort::P1,
                                    ColorConstants::kRed, PatternType::Amogus,
                                    false, 125, false);
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

frc2::CommandPtr LedSubsystem::setZoneColorPattern(LedZone zone, LedPort port,
                                                   frc::Color8Bit color,
                                                   PatternType pattern,
                                                   bool oneShot, int16_t delay,
                                                   bool reversed) {
  return frc2::DeferredCommand(
             [this, port, color, zone, pattern, oneShot, delay, reversed] {
               auto zoneIndex = static_cast<uint8_t>(zone);
               ConsoleWriter.logVerbose(
                   "ConnectorX",
                   "Attempting to set port %u zone %u to color=%s pattern=%u "
                   "reversed=%u",
                   static_cast<uint8_t>(port), zoneIndex,
                   color.HexString().c_str(), static_cast<uint8_t>(pattern),
                   reversed);
               return frc2::InstantCommand(
                          [this, port, color, zoneIndex] {
                            m_connectorX.setColor(port, color, zoneIndex);
                          },
                          {this})
                   .ToPtr()
                   .AndThen(frc2::WaitCommand(kConnectorXDelay).ToPtr())
                   .AndThen(frc2::InstantCommand(
                                [this, port, pattern, oneShot, delay, zoneIndex,
                                 reversed] {
                                  m_connectorX.setPattern(port, pattern,
                                                          oneShot, delay,
                                                          zoneIndex, reversed);
                                },
                                {this})
                                .ToPtr());
             },
             {this})
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::AimbotEnabled() {
  // Acid green

  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n",
                                 "AimbotEnabled");

           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kOrange, PatternType::Blink, false, 600);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kOrange, PatternType::Blink, false, 600);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kOrange, PatternType::Blink,
                                    false, 600);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kOrange, PatternType::Blink, false, 600);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kAcidGreen,
                                    PatternType::Blink, false, 300);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::OnTheFlyPP() {
  // Blue SetAll for duration of on the fly

  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "OnTheFlyPP");

           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kPurple, PatternType::Chase, false, 60, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kPurple, PatternType::Chase, false, 60, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kPurple, PatternType::Chase, false, 60, false);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kPurple, PatternType::Chase,
                                    false, 60, false);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::VisionNoteDetected() {
  // Chases orange
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "VisionNoteDetected");

           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kOrange, PatternType::Chase, true, 120);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kOrange, PatternType::Chase, true, 120);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kOrange, PatternType::Chase,
                                    true, 120);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::SuccessfulIntake() {
  // Flash LEDs green once
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s\n", "SuccessfulIntake");
           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::SetAll, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kGreen, PatternType::SetAll, true);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kGreen, PatternType::SetAll,
                                    true);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::AutoScoring() {
  return frc2::InstantCommand([this] {
           ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n",
                                 "AutoScoring");

           setZoneColorPatternAsync(
               LedZone::LeftClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Blink, false, 250);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::RightClimber, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Blink, false, 250);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                                    ColorConstants::kRed, PatternType::Blink,
                                    false, 250);
           delaySeconds(kConnectorXDelay);
           setZoneColorPatternAsync(
               LedZone::Front, LedConstants::kIntakeLedPort,
               ColorConstants::kRed, PatternType::Blink, false, 250);
           delaySeconds(kConnectorXDelay);
           syncAllZones();
         })
      .ToPtr();
}

void LedSubsystem::RampingAsync() {
  ConsoleWriter.logInfo("LedSubsystem", "Setting LEDs to %s\n", "Ramping");

  setZoneColorPatternAsync(LedZone::LeftClimber, LedConstants::kIntakeLedPort,
                           ColorConstants::kOrange, PatternType::Chase, false,
                           40, false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::RightClimber, LedConstants::kIntakeLedPort,
                           ColorConstants::kOrange, PatternType::Chase, false,
                           40, true);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                           ColorConstants::kOrange, PatternType::SineRoll,
                           false, 30, false);
  delaySeconds(kConnectorXDelay);
  setZoneColorPatternAsync(LedZone::Front, LedConstants::kIntakeLedPort,
                           ColorConstants::kOrange, PatternType::SineRoll,
                           false, 30, false);
  delaySeconds(kConnectorXDelay);
  m_connectorX.syncZones(LedConstants::kIntakeLedPort,
                         {
                             static_cast<uint8_t>(LedZone::LeftClimber),
                             static_cast<uint8_t>(LedZone::RightClimber),
                         });
}

void LedSubsystem::setZoneColorPatternAsync(LedZone zone, LedPort port,
                                            frc::Color8Bit color,
                                            PatternType pattern, bool oneShot,
                                            int16_t delay, bool reversed) {
  auto zoneIndex = static_cast<uint8_t>(zone);
  ConsoleWriter.logVerbose(
      "ConnectorX",
      "Attempting to set port %u zone %u to color=%s pattern=%u "
      "reversed=%u",
      static_cast<uint8_t>(port), zoneIndex, color.HexString().c_str(),
      static_cast<uint8_t>(pattern), reversed);

  m_connectorX.setPattern(port, pattern, oneShot, delay, zoneIndex, reversed);
  delaySeconds(kConnectorXDelay);
  m_connectorX.setColor(port, color, zoneIndex);
}

void LedSubsystem::delaySeconds(units::second_t delaySeconds) {
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void LedSubsystem::createZones(LedPort port,
                               std::vector<Commands::NewZone> &&zones) {
  m_connectorX.createZones(port, std::move(zones));
}

void LedSubsystem::showFace(EyePattern pattern) {
  setZoneColorPatternAsync(LedZone::Back, LedPort::P1, ColorConstants::kRed,
                           static_cast<PatternType>(pattern), false, 500,
                           false);
}

void LedSubsystem::syncAllZones() {
  ConsoleWriter.logInfo("LedSubsystem", "Syncing all zones %s", "");
  m_connectorX.syncZones(LedConstants::kIntakeLedPort,
                         {static_cast<uint8_t>(LedZone::LeftClimber),
                          static_cast<uint8_t>(LedZone::Back),
                          static_cast<uint8_t>(LedZone::RightClimber),
                          static_cast<uint8_t>(LedZone::Front)});
}