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
        //    ConsoleLogger::getInstance().logInfo(
        //        "LedSubsystem", "Setting LEDs to %s", "Intaking");
        //    setZoneColorPatternAsync(
        //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kRed, PatternType::Chase, false, 60, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kRed, PatternType::Chase, false, 60, true);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
        //                             ColorConstants::kRed, PatternType::Blink,
        //                             false, 400, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kRed, PatternType::Chase, false, 40, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kRed, PatternType::Chase, false, 40, true);
        //    delaySeconds(kConnectorXDelay);
        //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
        //                           {
        //                               (uint8_t)LedZone::LeftClimber,
        //                               (uint8_t)LedZone::RightClimber,
        //                           });
        //    delaySeconds(kConnectorXDelay);
        //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
        //                           {
        //                               (uint8_t)LedZone::LeftEye,
        //                               (uint8_t)LedZone::RightEye,
        //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Outaking() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s", "Outaking");
           //    setZoneColorPatternAsync(
           //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Chase, false, 60, true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Chase, false, 60,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(LedZone::Back,
           //    LedConstants::kIntakeLedPort,
           //                             ColorConstants::kRed,
           //                             PatternType::Blink, false, 400,
           //                             false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Chase, false, 40,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Chase, false, 40, true);
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftClimber,
           //                               (uint8_t)LedZone::RightClimber,
           //                           });
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftEye,
           //                               (uint8_t)LedZone::RightEye,
           //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSpeaker() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s", "ScoringSpeaker");
           //    setZoneColorPatternAsync(
           //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kPurple, PatternType::Chase, false, 100,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kPurple, PatternType::Chase, false, 100,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(LedZone::Back,
           //    LedConstants::kIntakeLedPort,
           //                             ColorConstants::kRed,
           //                             PatternType::SineRoll, false, 40,
           //                             false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Breathe, false, 30,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kRed, PatternType::Breathe, false, 30,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftClimber,
           //                               (uint8_t)LedZone::RightClimber,
           //                           });
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftEye,
           //                               (uint8_t)LedZone::RightEye,
           //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringAmp() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s", "ScoringAmp");
           //    setZoneColorPatternAsync(
           //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kTeal, PatternType::Chase, false, 100,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kTeal, PatternType::Chase, false, 100,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(LedZone::Back,
           //    LedConstants::kIntakeLedPort,
           //                             ColorConstants::kRed,
           //                             PatternType::Breathe, false, 30,
           //                             false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kTeal, PatternType::SineRoll, false, 40,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kTeal, PatternType::SineRoll, false, 40,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftClimber,
           //                               (uint8_t)LedZone::RightClimber,
           //                           });
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftEye,
           //                               (uint8_t)LedZone::RightEye,
           //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::ScoringSubwoofer() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s", "ScoringSubwoofer");
           //    setZoneColorPatternAsync(
           //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kYellow, PatternType::Chase, false, 100,
           //        false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
           //        ColorConstants::kYellow, PatternType::Chase, false, 100,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(LedZone::Back,
           //    LedConstants::kIntakeLedPort,
           //                             ColorConstants::kRed,
           //                             PatternType::Breathe, false, 30,
           //                             false);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(
           //        LedZone::RightEye, LedConstants::kIntakeLedPort,
           //        ColorConstants::kYellow, PatternType::SineRoll, false, 40,
           //        true);
           //    delaySeconds(kConnectorXDelay);
           //    setZoneColorPatternAsync(LedZone::LeftEye,
           //                             LedConstants::kIntakeLedPort,
           //                             ColorConstants::kYellow,
           //                             PatternType::SineRoll, false, 40,
           //                             false);
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftClimber,
           //                               (uint8_t)LedZone::RightClimber,
           //                           });
           //    delaySeconds(kConnectorXDelay);
           //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
           //                           {
           //                               (uint8_t)LedZone::LeftEye,
           //                               (uint8_t)LedZone::RightEye,
           //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Loaded() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo("LedSubsystem",
                                                "Setting LEDs to %s", "Loaded");
        //    setZoneColorPatternAsync(
        //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
        //                             ColorConstants::kRed, PatternType::SetAll,
        //                             true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Idling() {
  return frc2::InstantCommand([this] { IdlingAsync(); }).ToPtr();
}

void LedSubsystem::IdlingAsync() {
  ConsoleLogger::getInstance().logInfo("LedSubsystem", "Setting LEDs to %s",
                                       "Idling");
//   setZoneColorPatternAsync(LedZone::LeftClimber, LedConstants::kIdleLedPort,
//                            ColorConstants::kBlue, PatternType::Breathe, false,
//                            30, false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::RightClimber, LedConstants::kIdleLedPort,
//                            ColorConstants::kBlue, PatternType::Breathe, false,
//                            30, false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::Back, LedConstants::kIdleLedPort,
//                            ColorConstants::kBlue, PatternType::Breathe, false,
//                            30, false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::RightEye, LedConstants::kIdleLedPort,
//                            ColorConstants::kBlue, PatternType::Breathe, false,
//                            30, false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::LeftEye, LedConstants::kIdleLedPort,
//                            ColorConstants::kBlue, PatternType::Breathe, false,
//                            30, false);
//   delaySeconds(kConnectorXDelay);
//   syncAllZones();
}

frc2::CommandPtr LedSubsystem::Climbing() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo(
               "LedSubsystem", "Setting LEDs to %s", "Climbing");
        //    setZoneColorPatternAsync(
        //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::Chase, false, 200, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::Chase, false, 200, true);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
        //                             ColorConstants::kGreen,
        //                             PatternType::SineRoll, false, 50, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SineRoll, false, 50, true);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SineRoll, false, 50, false);
        //    delaySeconds(kConnectorXDelay);
        //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
        //                           {
        //                               (uint8_t)LedZone::LeftClimber,
        //                               (uint8_t)LedZone::RightClimber,
        //                           });
        //    delaySeconds(kConnectorXDelay);
        //    m_connectorX.syncZones(LedConstants::kIntakeLedPort,
        //                           {
        //                               (uint8_t)LedZone::LeftEye,
        //                               (uint8_t)LedZone::RightEye,
        //                           });
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Funni() {
  return frc2::InstantCommand([this] {
           ConsoleLogger::getInstance().logInfo("LedSubsystem",
                                                "Setting LEDs to %s", "Funni");
        //    setZoneColorPatternAsync(
        //        LedZone::LeftClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::RGBFade, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightClimber, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
        //                             ColorConstants::kRed, PatternType::SetAll,
        //                             true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::RightEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    setZoneColorPatternAsync(
        //        LedZone::LeftEye, LedConstants::kIntakeLedPort,
        //        ColorConstants::kGreen, PatternType::SetAll, true, -1, false);
        //    delaySeconds(kConnectorXDelay);
        //    syncAllZones();
         })
      .ToPtr();
}

frc2::CommandPtr LedSubsystem::Error() {
  return frc2::InstantCommand([this] { ErrorAsync(); }).ToPtr();
}

void LedSubsystem::ErrorAsync() {
  ConsoleLogger::getInstance().logInfo("LedSubsystem", "Setting LEDs to %s",
                                       "Error");
//   setZoneColorPatternAsync(LedZone::LeftClimber, LedConstants::kIntakeLedPort,
//                            ColorConstants::kRed, PatternType::Blink, false, 750,
//                            false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::RightClimber, LedConstants::kIntakeLedPort,
//                            ColorConstants::kRed, PatternType::Blink, false, 750,
//                            false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::Back, LedConstants::kIntakeLedPort,
//                            ColorConstants::kRed, PatternType::Blink, false, 750,
//                            false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::RightEye, LedConstants::kIntakeLedPort,
//                            ColorConstants::kRed, PatternType::SetAll, true, -1,
//                            false);
//   delaySeconds(kConnectorXDelay);
//   setZoneColorPatternAsync(LedZone::LeftEye, LedConstants::kIntakeLedPort,
//                            ColorConstants::kRed, PatternType::SetAll, true, -1,
//                            false);
//   delaySeconds(kConnectorXDelay);
//   m_connectorX.syncZones(LedConstants::kIntakeLedPort,
//                          {
//                              (uint8_t)LedZone::LeftClimber,
//                              (uint8_t)LedZone::RightClimber,
//                          });
//   delaySeconds(kConnectorXDelay);
//   m_connectorX.syncZones(LedConstants::kIntakeLedPort,
//                          {
//                              (uint8_t)LedZone::LeftEye,
//                              (uint8_t)LedZone::RightEye,
//                          });
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
//   std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void LedSubsystem::createZones(LedPort port,
                               std::vector<Commands::NewZone> &&zones) {
  m_connectorX.createZones(LedPort::P1, std::move(zones));
}

void LedSubsystem::syncAllZones() {
  ConsoleLogger::getInstance().logInfo("LedSubsystem", "Syncing all zones %s",
                                       "");
  m_connectorX.syncZones(
      LedConstants::kIntakeLedPort,
      {(uint8_t)LedZone::LeftClimber, (uint8_t)LedZone::Back,
       (uint8_t)LedZone::RightClimber, (uint8_t)LedZone::RightEye,
       (uint8_t)LedZone::LeftEye});
}
