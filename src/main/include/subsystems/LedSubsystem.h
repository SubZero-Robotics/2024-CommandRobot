#pragma once

#include <frc/Notifier.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>
#include <chrono>
#include <thread>

#include "Constants.h"
#include "moduledrivers/ConnectorX.h"

using namespace ConnectorX;

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem() : m_connectorX(ConnectorXBoard(kLedAddress)) {
    ConsoleLogger::getInstance().logVerbose("LedSubsystem", "LEDs init%s", "");
    createZones(LedPort::P1, std::move(m_ledZones));
    m_connectorX.setOn();
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

  void IdlingAsync();
  void ErrorAsync();

 private:
  enum class LedZone {
    LeftClimber = 0,
    Back,
    RightClimber,
    RightEye,
    LeftEye,
  };

  frc2::CommandPtr setZoneColorPattern(LedZone zone, LedPort port,
                                       frc::Color8Bit color,
                                       PatternType pattern,
                                       bool oneShot = false, int16_t delay = -1,
                                       bool reversed = false);

  void setZoneColorPatternAsync(LedZone zone, LedPort port,
                                frc::Color8Bit color, PatternType pattern,
                                bool oneShot = false, int16_t delay = -1,
                                bool reversed = false);

  void delaySeconds(units::second_t delaySeconds);

  void createZones(LedPort port, std::vector<Commands::NewZone> &&zones);

  void syncAllZones();

  ConnectorXBoard m_connectorX;
  // TODO: make into a constant elsewhere
  const uint16_t m_totalLeds = 41;
  std::vector<Commands::NewZone> m_ledZones = {
      {.offset = 0, .count = 2048},    {.offset = 2048, .count = 3840},
      {.offset = 5888, .count = 2048}, {.offset = 7936, .count = 1280},
      {.offset = 9216, .count = 1280},
  };
};