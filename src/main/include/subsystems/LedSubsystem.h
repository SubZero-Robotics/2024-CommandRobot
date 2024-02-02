#pragma once

#include <frc/util/Color8Bit.h>
#include <frc2/command/DeferredCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>

#include "Constants.h"
#include "moduledrivers/ConnectorX.h"

using namespace ConnectorX;

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem() : m_connectorX(ConnectorXBoard(kLedAddress)) {
    ConsoleLogger::getInstance().logInfo("Led Sussystem", "Setting zones %s",
                                         "");
    createZones(LedPort::P1, std::move(m_ledZones));
    m_connectorX.setOn();
  }

  void Periodic() override;

  void SimulationPeriodic() override;

  frc2::DeferredCommand GetDeferredFromState(StateGetter);
  frc2::CommandPtr ShowFromState(StateGetter);

  frc2::CommandPtr Intaking();
  frc2::CommandPtr ScoringSpeaker();
  frc2::CommandPtr ScoringAmp();
  frc2::CommandPtr Loaded();
  frc2::CommandPtr Idling();
  frc2::CommandPtr Climbing();
  frc2::CommandPtr Error();

 private:
  enum class LedZone {
    Back = 0,
    Right,
    Front,
    Left,
  };

  frc2::CommandPtr setZoneColorPattern(LedZone zone, LedPort port,
                                       frc::Color8Bit color,
                                       PatternType pattern,
                                       bool oneShot = false, int16_t delay = -1,
                                       bool reversed = false);

  void createZones(LedPort port, std::vector<Commands::NewZone> &&zones);

  frc2::CommandPtr syncAllZones();

  ConnectorXBoard m_connectorX;
  std::vector<Commands::NewZone> m_ledZones = {
      {.offset = 0, .count = 15},
      {.offset = 15, .count = 15},
      {.offset = 30, .count = 15},
      {.offset = 45, .count = 15},
  };
};