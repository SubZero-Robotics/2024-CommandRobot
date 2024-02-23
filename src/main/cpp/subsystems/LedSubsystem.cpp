#include "subsystems/LedSubsystem.h"

#include <frc/Timer.h>

#include "ColorConstants.h"
#include "utils/ConsoleLogger.h"

void LedSubsystem::Periodic() {}

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
           m_notifier.Stop();
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "Intaking");
             setZoneColorPatternAsync(
                 LedZone::Left, LedConstants::kIntakeLedPort,
                 ColorConstants::kRed, PatternType::Chase, false, 60, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Right, LedConstants::kIntakeLedPort,
                 ColorConstants::kRed, PatternType::Chase, false, 60, true);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Front, LedConstants::kIntakeLedPort,
                 ColorConstants::kRed, PatternType::SineRoll, false, 40, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Back, LedConstants::kIntakeLedPort,
                 ColorConstants::kRed, PatternType::SineRoll, false, 40, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
  return frc2::InstantCommand([this] {
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "ScoringSpeaker");
             setZoneColorPatternAsync(LedZone::Left,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Right,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Front,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Back,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kPurple,
                                      PatternType::Blink, false, 250, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
  return frc2::InstantCommand([this] {
           m_notifier.Stop();
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "ScoringAmp");
             setZoneColorPatternAsync(LedZone::Left,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Right,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Front,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kPurple,
                                      PatternType::Blink, false, 250, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Back,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::SetAll, true, 500, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Loaded() {
  return frc2::InstantCommand([this] {
           m_notifier.Stop();
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "Loaded");
             setZoneColorPatternAsync(LedZone::Left, LedConstants::kStowLedPort,
                                      ColorConstants::kGreen,
                                      PatternType::Blink, false, 750, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Right, LedConstants::kStowLedPort,
                 ColorConstants::kGreen, PatternType::Blink, false, 750, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Front, LedConstants::kStowLedPort,
                 ColorConstants::kGreen, PatternType::Blink, false, 750, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Back, LedConstants::kStowLedPort,
                                      ColorConstants::kGreen,
                                      PatternType::Blink, false, 750, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Idling() {
  return frc2::InstantCommand([this] { IdlingAsync(); }).ToPtr();
}

void LedSubsystem::IdlingAsync() {
  m_notifier.Stop();
  m_notifier.SetCallback([this] {
    ConsoleLogger::getInstance().logInfo("LedSubsystem", "Setting LEDs to %s",
                                         "Idling");
    setZoneColorPatternAsync(LedZone::Left, LedConstants::kIdleLedPort,
                             ColorConstants::kBlue, PatternType::Breathe, false,
                             20, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Right, LedConstants::kIdleLedPort,
                             ColorConstants::kBlue, PatternType::Breathe, false,
                             20, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Front, LedConstants::kIdleLedPort,
                             ColorConstants::kBlue, PatternType::Breathe, false,
                             20, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIdleLedPort,
                             ColorConstants::kBlue, PatternType::Breathe, false,
                             20, false);
    delaySeconds(kConnectorXDelay);
    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
                           {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                            (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
  });
  m_notifier.StartSingle(0_s);
}

frc2::CommandPtr LedSubsystem::Climbing() {
  return frc2::InstantCommand([this] {
           m_notifier.Stop();
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "Climbing");
             setZoneColorPatternAsync(LedZone::Left, LedConstants::kIdleLedPort,
                                      ColorConstants::kGreen,
                                      PatternType::Chase, false, 50, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Right, LedConstants::kIdleLedPort,
                 ColorConstants::kGreen, PatternType::Chase, false, 50, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(
                 LedZone::Front, LedConstants::kIdleLedPort,
                 ColorConstants::kGreen, PatternType::Chase, false, 50, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Back, LedConstants::kIdleLedPort,
                                      ColorConstants::kGreen,
                                      PatternType::Chase, false, 50, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Funni() {
  return frc2::InstantCommand([this] {
           m_notifier.SetCallback([this] {
             ConsoleLogger::getInstance().logInfo(
                 "LedSubsystem", "Setting LEDs to %s", "Funni");
             setZoneColorPatternAsync(LedZone::Left,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::RGBFade, true, 10, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Right,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::RGBFade, true, 10, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Front,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kYellow,
                                      PatternType::RGBFade, true, 10, false);
             delaySeconds(kConnectorXDelay);
             setZoneColorPatternAsync(LedZone::Back,
                                      LedConstants::kSpeakerLedPort,
                                      ColorConstants::kPurple,
                                      PatternType::RGBFade, true, 10, false);
             delaySeconds(kConnectorXDelay);
             m_connectorX.syncZones(
                 LedConstants::kIntakeLedPort,
                 {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                  (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
           });
           m_notifier.StartSingle(0_s);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Error() {
  return frc2::InstantCommand([this] { ErrorAsync(); }).ToPtr();
}

void LedSubsystem::ErrorAsync() {
  m_notifier.Stop();
  m_notifier.SetCallback([this] {
    ConsoleLogger::getInstance().logInfo("LedSubsystem", "Setting LEDs to %s",
                                         "Error");
    setZoneColorPatternAsync(LedZone::Left, LedConstants::kIntakeLedPort,
                             ColorConstants::kRed, PatternType::Blink, false,
                             400, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Right, LedConstants::kIntakeLedPort,
                             ColorConstants::kRed, PatternType::Blink, false,
                             400, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Front, LedConstants::kIntakeLedPort,
                             ColorConstants::kRed, PatternType::Blink, false,
                             400, false);
    delaySeconds(kConnectorXDelay);
    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
                             ColorConstants::kRed, PatternType::Blink, false,
                             400, false);
    delaySeconds(0.1_s);
    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
                           {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                            (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
  });
  m_notifier.StartSingle(0_s);
}

frc2::CommandPtr LedSubsystem::setZoneColorPattern(LedZone zone, LedPort port,
                                                   frc::Color8Bit color,
                                                   PatternType pattern,
                                                   bool oneShot, int16_t delay,
                                                   bool reversed) {
  return frc2::DeferredCommand(
             [this, port, color, zone, pattern, oneShot, delay, reversed] {
               auto zoneIndex = static_cast<uint8_t>(zone);
               ConsoleLogger::getInstance().logVerbose(
                   "ConnectorX",
                   "Attempting to set port %u zone %u to color=%s pattern=%u "
                   "reversed=%u",
                   (uint8_t)port, zoneIndex, color.HexString().c_str(),
                   (uint8_t)pattern, reversed);
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

void LedSubsystem::setZoneColorPatternAsync(LedZone zone, LedPort port,
                                            frc::Color8Bit color,
                                            PatternType pattern, bool oneShot,
                                            int16_t delay, bool reversed) {
  auto zoneIndex = static_cast<uint8_t>(zone);
  m_connectorX.setColor(port, color, zoneIndex);
  ConsoleLogger::getInstance().logVerbose(
      "ConnectorX",
      "Attempting to set port %u zone %u to color=%s pattern=%u "
      "reversed=%u",
      (uint8_t)port, zoneIndex, color.HexString().c_str(), (uint8_t)pattern,
      reversed);
  delaySeconds(kConnectorXDelay);
  m_connectorX.setPattern(port, pattern, oneShot, delay, zoneIndex, reversed);
}

void LedSubsystem::delaySeconds(units::second_t delaySeconds) {
  frc::Timer timer;
  timer.Start();
  while (!timer.HasElapsed(delaySeconds))
    ;
  timer.Stop();
}

void LedSubsystem::createZones(LedPort port,
                               std::vector<Commands::NewZone> &&zones) {
  m_connectorX.createZones(LedPort::P1, std::move(zones));
}

frc2::CommandPtr LedSubsystem::syncAllZones() {
  return frc2::InstantCommand([] {
           ConsoleLogger::getInstance().logInfo("LedSubsystem",
                                                "Syncing all zones %s", "");
         })
      .ToPtr()
      .AndThen(frc2::InstantCommand([this] {
                 m_connectorX.syncZones(
                     LedConstants::kIntakeLedPort,
                     {(uint8_t)LedZone::Left, (uint8_t)LedZone::Right,
                      (uint8_t)LedZone::Front, (uint8_t)LedZone::Back});
               }).ToPtr());
}
